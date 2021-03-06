/*
 * Copyright 2002-2004, Instant802 Networks, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/netdevice.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/compiler.h>
#include <linux/ieee80211.h>
#include <asm/unaligned.h>
#include <net/mac80211.h>

#include "ieee80211_i.h"
#include "michael.h"
#include "tkip.h"
#include "aes_ccm.h"
#include "wpa.h"

ieee80211_tx_result
ieee80211_tx_h_michael_mic_add(struct ieee80211_tx_data *tx)
{
	u8 *data, *key, *mic, key_offset;
	size_t data_len;
	unsigned int hdrlen;
	struct ieee80211_hdr *hdr;
	struct sk_buff *skb = tx->skb;
	int authenticator;
	int wpa_test = 0;
	int tail;

	hdr = (struct ieee80211_hdr *)skb->data;
	if (!tx->key || tx->key->conf.alg != ALG_TKIP || skb->len < 24 ||
	    !ieee80211_is_data_present(hdr->frame_control))
		return TX_CONTINUE;

	hdrlen = ieee80211_hdrlen(hdr->frame_control);
	if (skb->len < hdrlen)
		return TX_DROP;

	data = skb->data + hdrlen;
	data_len = skb->len - hdrlen;

	if ((tx->key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE) &&
	    !(tx->flags & IEEE80211_TX_FRAGMENTED) &&
	    !(tx->key->conf.flags & IEEE80211_KEY_FLAG_GENERATE_MMIC) &&
	    !wpa_test) {
		/* hwaccel - with no need for preallocated room for Michael MIC
		 */
		return TX_CONTINUE;
	}

	tail = MICHAEL_MIC_LEN;
	if (!(tx->key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE))
		tail += TKIP_ICV_LEN;

	if (WARN_ON(skb_tailroom(skb) < tail ||
		    skb_headroom(skb) < TKIP_IV_LEN))
		return TX_DROP;

#if 0
	authenticator = fc & IEEE80211_FCTL_FROMDS; /* FIX */
#else
	authenticator = 1;
#endif
	/* At this point we know we're using ALG_TKIP. To get the MIC key
	 * we now will rely on the offset from the ieee80211_key_conf::key */
	key_offset = authenticator ?
		NL80211_TKIP_DATA_OFFSET_TX_MIC_KEY :
		NL80211_TKIP_DATA_OFFSET_RX_MIC_KEY;
	key = &tx->key->conf.key[key_offset];
	mic = skb_put(skb, MICHAEL_MIC_LEN);
	michael_mic(key, hdr, data, data_len, mic);

	return TX_CONTINUE;
}


ieee80211_rx_result
ieee80211_rx_h_michael_mic_verify(struct ieee80211_rx_data *rx)
{
	u8 *data, *key = NULL, key_offset;
	size_t data_len;
	unsigned int hdrlen;
	struct ieee80211_hdr *hdr;
	u8 mic[MICHAEL_MIC_LEN];
	struct sk_buff *skb = rx->skb;
	int authenticator = 1, wpa_test = 0;
	DECLARE_MAC_BUF(mac);

	/*
	 * No way to verify the MIC if the hardware stripped it
	 */
	if (rx->status->flag & RX_FLAG_MMIC_STRIPPED)
		return RX_CONTINUE;

	hdr = (struct ieee80211_hdr *)skb->data;
	if (!rx->key || rx->key->conf.alg != ALG_TKIP ||
	    !ieee80211_has_protected(hdr->frame_control) ||
	    !ieee80211_is_data_present(hdr->frame_control))
		return RX_CONTINUE;

	hdrlen = ieee80211_hdrlen(hdr->frame_control);
	if (skb->len < hdrlen + MICHAEL_MIC_LEN)
		return RX_DROP_UNUSABLE;

	data = skb->data + hdrlen;
	data_len = skb->len - hdrlen - MICHAEL_MIC_LEN;

#if 0
	authenticator = fc & IEEE80211_FCTL_TODS; /* FIX */
#else
	authenticator = 1;
#endif
	/* At this point we know we're using ALG_TKIP. To get the MIC key
	 * we now will rely on the offset from the ieee80211_key_conf::key */
	key_offset = authenticator ?
		NL80211_TKIP_DATA_OFFSET_RX_MIC_KEY :
		NL80211_TKIP_DATA_OFFSET_TX_MIC_KEY;
	key = &rx->key->conf.key[key_offset];
	michael_mic(key, hdr, data, data_len, mic);
	if (memcmp(mic, data + data_len, MICHAEL_MIC_LEN) != 0 || wpa_test) {
		if (!(rx->flags & IEEE80211_RX_RA_MATCH))
			return RX_DROP_UNUSABLE;

		mac80211_ev_michael_mic_failure(rx->sdata, rx->key->conf.keyidx,
						(void *) skb->data);
		return RX_DROP_UNUSABLE;
	}

	/* remove Michael MIC from payload */
	skb_trim(skb, skb->len - MICHAEL_MIC_LEN);

	/* update IV in key information to be able to detect replays */
	rx->key->u.tkip.rx[rx->queue].iv32 = rx->tkip_iv32;
	rx->key->u.tkip.rx[rx->queue].iv16 = rx->tkip_iv16;

	return RX_CONTINUE;
}


static int tkip_encrypt_skb(struct ieee80211_tx_data *tx, struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	struct ieee80211_key *key = tx->key;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	unsigned int hdrlen;
	int len, tail;
	u8 *pos;

	if ((tx->key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE) &&
	    !(tx->key->conf.flags & IEEE80211_KEY_FLAG_GENERATE_IV)) {
		/* hwaccel - with no need for preallocated room for IV/ICV */
		info->control.hw_key = &tx->key->conf;
		return 0;
	}

	hdrlen = ieee80211_hdrlen(hdr->frame_control);
	len = skb->len - hdrlen;

	if (tx->key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE)
		tail = 0;
	else
		tail = TKIP_ICV_LEN;

	if (WARN_ON(skb_tailroom(skb) < tail ||
		    skb_headroom(skb) < TKIP_IV_LEN))
		return -1;

	pos = skb_push(skb, TKIP_IV_LEN);
	memmove(pos, pos + TKIP_IV_LEN, hdrlen);
	pos += hdrlen;

	/* Increase IV for the frame */
	key->u.tkip.tx.iv16++;
	if (key->u.tkip.tx.iv16 == 0)
		key->u.tkip.tx.iv32++;

	if (tx->key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE) {
		/* hwaccel - with preallocated room for IV */
		ieee80211_tkip_add_iv(pos, key, key->u.tkip.tx.iv16);

		info->control.hw_key = &tx->key->conf;
		return 0;
	}

	/* Add room for ICV */
	skb_put(skb, TKIP_ICV_LEN);

	hdr = (struct ieee80211_hdr *) skb->data;
	ieee80211_tkip_encrypt_data(tx->local->wep_tx_tfm,
				    key, pos, len, hdr->addr2);
	return 0;
}


ieee80211_tx_result
ieee80211_crypto_tkip_encrypt(struct ieee80211_tx_data *tx)
{
	struct sk_buff *skb = tx->skb;

	ieee80211_tx_set_protected(tx);

	if (tkip_encrypt_skb(tx, skb) < 0)
		return TX_DROP;

	if (tx->extra_frag) {
		int i;
		for (i = 0; i < tx->num_extra_frag; i++) {
			if (tkip_encrypt_skb(tx, tx->extra_frag[i]) < 0)
				return TX_DROP;
		}
	}

	return TX_CONTINUE;
}


ieee80211_rx_result
ieee80211_crypto_tkip_decrypt(struct ieee80211_rx_data *rx)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) rx->skb->data;
	int hdrlen, res, hwaccel = 0, wpa_test = 0;
	struct ieee80211_key *key = rx->key;
	struct sk_buff *skb = rx->skb;
	DECLARE_MAC_BUF(mac);

	hdrlen = ieee80211_hdrlen(hdr->frame_control);

	if (!ieee80211_is_data(hdr->frame_control))
		return RX_CONTINUE;

	if (!rx->sta || skb->len - hdrlen < 12)
		return RX_DROP_UNUSABLE;

	if (rx->status->flag & RX_FLAG_DECRYPTED) {
		if (rx->status->flag & RX_FLAG_IV_STRIPPED) {
			/*
			 * Hardware took care of all processing, including
			 * replay protection, and stripped the ICV/IV so
			 * we cannot do any checks here.
			 */
			return RX_CONTINUE;
		}

		/* let TKIP code verify IV, but skip decryption */
		hwaccel = 1;
	}

	res = ieee80211_tkip_decrypt_data(rx->local->wep_rx_tfm,
					  key, skb->data + hdrlen,
					  skb->len - hdrlen, rx->sta->sta.addr,
					  hdr->addr1, hwaccel, rx->queue,
					  &rx->tkip_iv32,
					  &rx->tkip_iv16);
	if (res != TKIP_DECRYPT_OK || wpa_test)
		return RX_DROP_UNUSABLE;

	/* Trim ICV */
	skb_trim(skb, skb->len - TKIP_ICV_LEN);

	/* Remove IV */
	memmove(skb->data + TKIP_IV_LEN, skb->data, hdrlen);
	skb_pull(skb, TKIP_IV_LEN);

	return RX_CONTINUE;
}


static void ccmp_special_blocks(struct sk_buff *skb, u8 *pn, u8 *scratch,
				int encrypted)
{
	__le16 mask_fc;
	int a4_included;
	u8 qos_tid;
	u8 *b_0, *aad;
	u16 data_len, len_a;
	unsigned int hdrlen;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;

	b_0 = scratch + 3 * AES_BLOCK_LEN;
	aad = scratch + 4 * AES_BLOCK_LEN;

	/*
	 * Mask FC: zero subtype b4 b5 b6
	 * Retry, PwrMgt, MoreData; set Protected
	 */
	mask_fc = hdr->frame_control;
	mask_fc &= ~cpu_to_le16(0x0070 | IEEE80211_FCTL_RETRY |
				IEEE80211_FCTL_PM | IEEE80211_FCTL_MOREDATA);
	mask_fc |= cpu_to_le16(IEEE80211_FCTL_PROTECTED);

	hdrlen = ieee80211_hdrlen(hdr->frame_control);
	len_a = hdrlen - 2;
	a4_included = ieee80211_has_a4(hdr->frame_control);

	if (ieee80211_is_data_qos(hdr->frame_control))
		qos_tid = *ieee80211_get_qos_ctl(hdr) & IEEE80211_QOS_CTL_TID_MASK;
	else
		qos_tid = 0;

	data_len = skb->len - hdrlen - CCMP_HDR_LEN;
	if (encrypted)
		data_len -= CCMP_MIC_LEN;

	/* First block, b_0 */
	b_0[0] = 0x59; /* flags: Adata: 1, M: 011, L: 001 */
	/* Nonce: QoS Priority | A2 | PN */
	b_0[1] = qos_tid;
	memcpy(&b_0[2], hdr->addr2, ETH_ALEN);
	memcpy(&b_0[8], pn, CCMP_PN_LEN);
	/* l(m) */
	put_unaligned_be16(data_len, &b_0[14]);

	/* AAD (extra authenticate-only data) / masked 802.11 header
	 * FC | A1 | A2 | A3 | SC | [A4] | [QC] */
	put_unaligned_be16(len_a, &aad[0]);
	put_unaligned(mask_fc, (__le16 *)&aad[2]);
	memcpy(&aad[4], &hdr->addr1, 3 * ETH_ALEN);

	/* Mask Seq#, leave Frag# */
	aad[22] = *((u8 *) &hdr->seq_ctrl) & 0x0f;
	aad[23] = 0;

	if (a4_included) {
		memcpy(&aad[24], hdr->addr4, ETH_ALEN);
		aad[30] = qos_tid;
		aad[31] = 0;
	} else {
		memset(&aad[24], 0, ETH_ALEN + IEEE80211_QOS_CTL_LEN);
		aad[24] = qos_tid;
	}
}


static inline void ccmp_pn2hdr(u8 *hdr, u8 *pn, int key_id)
{
	hdr[0] = pn[5];
	hdr[1] = pn[4];
	hdr[2] = 0;
	hdr[3] = 0x20 | (key_id << 6);
	hdr[4] = pn[3];
	hdr[5] = pn[2];
	hdr[6] = pn[1];
	hdr[7] = pn[0];
}


static inline int ccmp_hdr2pn(u8 *pn, u8 *hdr)
{
	pn[0] = hdr[7];
	pn[1] = hdr[6];
	pn[2] = hdr[5];
	pn[3] = hdr[4];
	pn[4] = hdr[1];
	pn[5] = hdr[0];
	return (hdr[3] >> 6) & 0x03;
}


static int ccmp_encrypt_skb(struct ieee80211_tx_data *tx, struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	struct ieee80211_key *key = tx->key;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	int hdrlen, len, tail;
	u8 *pos, *pn;
	int i;

	if ((tx->key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE) &&
	    !(tx->key->conf.flags & IEEE80211_KEY_FLAG_GENERATE_IV)) {
		/* hwaccel - with no need for preallocated room for CCMP "
		 * header or MIC fields */
		info->control.hw_key = &tx->key->conf;
		return 0;
	}

	hdrlen = ieee80211_hdrlen(hdr->frame_control);
	len = skb->len - hdrlen;

	if (key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE)
		tail = 0;
	else
		tail = CCMP_MIC_LEN;

	if (WARN_ON(skb_tailroom(skb) < tail ||
		    skb_headroom(skb) < CCMP_HDR_LEN))
		return -1;

	pos = skb_push(skb, CCMP_HDR_LEN);
	memmove(pos, pos + CCMP_HDR_LEN, hdrlen);
	hdr = (struct ieee80211_hdr *) pos;
	pos += hdrlen;

	/* PN = PN + 1 */
	pn = key->u.ccmp.tx_pn;

	for (i = CCMP_PN_LEN - 1; i >= 0; i--) {
		pn[i]++;
		if (pn[i])
			break;
	}

	ccmp_pn2hdr(pos, pn, key->conf.keyidx);

	if (key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE) {
		/* hwaccel - with preallocated room for CCMP header */
		info->control.hw_key = &tx->key->conf;
		return 0;
	}

	pos += CCMP_HDR_LEN;
	ccmp_special_blocks(skb, pn, key->u.ccmp.tx_crypto_buf, 0);
	ieee80211_aes_ccm_encrypt(key->u.ccmp.tfm, key->u.ccmp.tx_crypto_buf, pos, len,
				  pos, skb_put(skb, CCMP_MIC_LEN));

	return 0;
}


ieee80211_tx_result
ieee80211_crypto_ccmp_encrypt(struct ieee80211_tx_data *tx)
{
	struct sk_buff *skb = tx->skb;

	ieee80211_tx_set_protected(tx);

	if (ccmp_encrypt_skb(tx, skb) < 0)
		return TX_DROP;

	if (tx->extra_frag) {
		int i;
		for (i = 0; i < tx->num_extra_frag; i++) {
			if (ccmp_encrypt_skb(tx, tx->extra_frag[i]) < 0)
				return TX_DROP;
		}
	}

	return TX_CONTINUE;
}


ieee80211_rx_result
ieee80211_crypto_ccmp_decrypt(struct ieee80211_rx_data *rx)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)rx->skb->data;
	int hdrlen;
	struct ieee80211_key *key = rx->key;
	struct sk_buff *skb = rx->skb;
	u8 pn[CCMP_PN_LEN];
	int data_len;
	DECLARE_MAC_BUF(mac);

	hdrlen = ieee80211_hdrlen(hdr->frame_control);

	if (!ieee80211_is_data(hdr->frame_control))
		return RX_CONTINUE;

	data_len = skb->len - hdrlen - CCMP_HDR_LEN - CCMP_MIC_LEN;
	if (!rx->sta || data_len < 0)
		return RX_DROP_UNUSABLE;

	if ((rx->status->flag & RX_FLAG_DECRYPTED) &&
	    (rx->status->flag & RX_FLAG_IV_STRIPPED))
		return RX_CONTINUE;

	(void) ccmp_hdr2pn(pn, skb->data + hdrlen);

	if (memcmp(pn, key->u.ccmp.rx_pn[rx->queue], CCMP_PN_LEN) <= 0) {
		key->u.ccmp.replays++;
		return RX_DROP_UNUSABLE;
	}

	if (!(rx->status->flag & RX_FLAG_DECRYPTED)) {
		/* hardware didn't decrypt/verify MIC */
		ccmp_special_blocks(skb, pn, key->u.ccmp.rx_crypto_buf, 1);

		if (ieee80211_aes_ccm_decrypt(
			    key->u.ccmp.tfm, key->u.ccmp.rx_crypto_buf,
			    skb->data + hdrlen + CCMP_HDR_LEN, data_len,
			    skb->data + skb->len - CCMP_MIC_LEN,
			    skb->data + hdrlen + CCMP_HDR_LEN)) {
			return RX_DROP_UNUSABLE;
		}
	}

	memcpy(key->u.ccmp.rx_pn[rx->queue], pn, CCMP_PN_LEN);

	/* Remove CCMP header and MIC */
	skb_trim(skb, skb->len - CCMP_MIC_LEN);
	memmove(skb->data + CCMP_HDR_LEN, skb->data, hdrlen);
	skb_pull(skb, CCMP_HDR_LEN);

	return RX_CONTINUE;
}
