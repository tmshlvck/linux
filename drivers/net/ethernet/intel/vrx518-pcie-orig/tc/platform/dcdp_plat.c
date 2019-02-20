/*******************************************************************************

  Intel SmartPHY DSL PCIe TC driver
  Copyright(c) 2016 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

*******************************************************************************/
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/atomic.h>
#include <linux/uaccess.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/seq_file.h>
#include <linux/printk.h>
#include <linux/etherdevice.h>
#include <linux/workqueue.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include "../inc/dsl_tc.h"
#include <net/directconnect_dp_api.h>

#include "../inc/tc_main.h"
#include "../inc/reg_addr.h"

enum {
	US_BOND = 0,
	DS_BOND = 1,
	BOND_MAX,
};

#define PMAC_SIZE		8
#define DS_DEF_DESC_NUM		2048

/* PMAC configuration */
#define PMAC_SPPID		0xF
#define PMAC_SPPID_S		4
#define PMAC_PMAP_EN		0x1
#define PMAC_PMAP_EN_S		0x7
#define PMAC_FCS_INS		0x1
#define PMAC_FCS_INS_S		0x7

/* skb port configuration */
#define SKB_PORT_ID		0xF
#define SKB_PORT_ID_S		8
#define SKB_PORT_SUBID		0x7FFF
#define SKB_PORT_SUBID_S	0

struct tc_req {
	struct work_struct work;
	u32 id;
	enum dsl_tc_mode tc_mode;
};

struct plat_priv {
	struct tc_priv *tc_priv;
	struct module *owner;
	u32 port_id;
	struct dc_dp_cb cb;
	struct dc_dp_res soc_res;
	struct dc_dp_dev dev;
	struct ltq_mei_atm_showtime_info dsl_ops;
	struct tc_req req_work;
};

static struct plat_priv *g_plat_priv;
static struct module tc_mod;
#ifndef CONFIG_SOC_TYPE_XWAY
void *ppa_callback_get(e_ltq_mei_cb_type type)
{
	struct plat_priv *priv;

	priv = g_plat_priv;
	if (WARN_ON(priv == NULL))
		return NULL;

	switch (type) {
	case LTQ_MEI_SHOWTIME_CHECK:
		return priv->dsl_ops.check_ptr;
	case LTQ_MEI_SHOWTIME_ENTER:
		return priv->dsl_ops.enter_ptr;
	case LTQ_MEI_SHOWTIME_EXIT:
		return priv->dsl_ops.exit_ptr;
	case LTQ_MEI_TC_REQUEST:
		return priv->dsl_ops.req_tc_ptr;
	case LTQ_MEI_TC_RESET:
		return priv->dsl_ops.tc_reset_ptr;
	case LTQ_MEI_ERB_ADDR_GET:
		return priv->dsl_ops.erb_addr_ptr;

	default:
		tc_err(priv->tc_priv, MSG_INIT,
			"get mei unknown function type %d\n", type);
		return NULL;
	}
}
EXPORT_SYMBOL(ppa_callback_get);

int ppa_callback_set(e_ltq_mei_cb_type type, void *func)
{
	struct plat_priv *priv;

	priv = g_plat_priv;
	if (WARN_ON(priv == NULL))
		return -ENODEV;

	switch (type) {
	/* save func address within global struct */
	case LTQ_MEI_SHOWTIME_CHECK:
		priv->dsl_ops.check_ptr = func;
		break;
	case LTQ_MEI_SHOWTIME_ENTER:
		priv->dsl_ops.enter_ptr = func;
		break;
	case LTQ_MEI_SHOWTIME_EXIT:
		priv->dsl_ops.exit_ptr = func;
		break;
	case LTQ_MEI_TC_REQUEST:
		priv->dsl_ops.req_tc_ptr = func;
		break;
	case LTQ_MEI_TC_RESET:
		priv->dsl_ops.tc_reset_ptr = func;
		break;
	case LTQ_MEI_ERB_ADDR_GET:
		priv->dsl_ops.erb_addr_ptr = func;
		break;

	default:
		tc_err(priv->tc_priv, MSG_INIT,
			"set mei unknown function type: %d\n", type);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(ppa_callback_set);
#endif
size_t plat_priv_sz(void)
{
	return sizeof(struct plat_priv);
}

static inline void *tc_plat_priv(struct tc_priv *priv)
{
	return (char *)priv + ALIGN(sizeof(struct tc_priv), TCPRIV_ALIGN);
}

static inline struct tc_priv *plat_to_tcpriv(void)
{
	return g_plat_priv->tc_priv;
}

static inline struct device *get_plat_device(void)
{
	return g_plat_priv->tc_priv->ep_dev[0].dev;
}

#if 1
#define DC_DP_DBG pr_debug
static void inline dc_dp_dump_dccntr_mode(int32_t dccntr_mode, char *ret)
{
	ret[0] = '\0';

	if (dccntr_mode & DC_DP_F_DCCNTR_MODE_INCREMENTAL)
		strcat(ret, "INCREMENTAL:");
	if (dccntr_mode & DC_DP_F_DCCNTR_MODE_CUMULATIVE)
		strcat(ret, "CUMULATIVE:");
	if (dccntr_mode & DC_DP_F_DCCNTR_MODE_LITTLE_ENDIAN)
		strcat(ret, "LE:");
	if (dccntr_mode & DC_DP_F_DCCNTR_MODE_BIG_ENDIAN)
		strcat(ret, "BE:");

	ret[strlen(ret)] = '\0';
}

static void inline dc_dp_dump_dccntr(struct dc_dp_dccntr *dccntr)
{

	char soc_mode_str[128];
	char dev_mode_str[128];

	dc_dp_dump_dccntr_mode(dccntr->soc_write_dccntr_mode, soc_mode_str);
	dc_dp_dump_dccntr_mode(dccntr->dev_write_dccntr_mode, dev_mode_str);

	DC_DP_DBG("soc_write_dccntr_mode      [%s]\n", soc_mode_str);
	DC_DP_DBG("dev_write_dccntr_mode      [%s]\n", dev_mode_str);
	DC_DP_DBG("soc2dev_enq_phys_base      [0x%pK]\n", dccntr->soc2dev_enq_phys_base);
	DC_DP_DBG("soc2dev_enq_base           [0x%pK]\n", dccntr->soc2dev_enq_base);
	DC_DP_DBG("soc2dev_enq_dccntr_len     [%d]\n", dccntr->soc2dev_enq_dccntr_len);
	DC_DP_DBG("soc2dev_ret_deq_phys_base  [0x%pK]\n", dccntr->soc2dev_ret_deq_phys_base);
	DC_DP_DBG("soc2dev_ret_deq_base       [0x%pK]\n", dccntr->soc2dev_ret_deq_base);
	DC_DP_DBG("soc2dev_ret_deq_dccntr_len [%d]\n", dccntr->soc2dev_ret_deq_dccntr_len);
	DC_DP_DBG("dev2soc_deq_phys_base      [0x%pK]\n", dccntr->dev2soc_deq_phys_base);
	DC_DP_DBG("dev2soc_deq_base           [0x%pK]\n", dccntr->dev2soc_deq_base);
	DC_DP_DBG("dev2soc_deq_dccntr_len     [%d]\n", dccntr->dev2soc_deq_dccntr_len);
	DC_DP_DBG("dev2soc_ret_enq_phys_base  [0x%pK]\n", dccntr->dev2soc_ret_enq_phys_base);
	DC_DP_DBG("dev2soc_ret_enq_base       [0x%pK]\n", dccntr->dev2soc_ret_enq_base);
	DC_DP_DBG("dev2soc_ret_enq_dccntr_len [%d]\n", dccntr->dev2soc_ret_enq_dccntr_len);
	DC_DP_DBG("soc2dev_deq_phys_base      [0x%pK]\n", dccntr->soc2dev_deq_phys_base);
	DC_DP_DBG("soc2dev_deq_base           [0x%pK]\n", dccntr->soc2dev_deq_base);
	DC_DP_DBG("soc2dev_deq_dccntr_len     [%d]\n", dccntr->soc2dev_deq_dccntr_len);
	DC_DP_DBG("soc2dev_ret_enq_phys_base  [0x%pK]\n", dccntr->soc2dev_ret_enq_phys_base);
	DC_DP_DBG("soc2dev_ret_enq_base       [0x%pK]\n", dccntr->soc2dev_ret_enq_base);
	DC_DP_DBG("soc2dev_ret_enq_dccntr_len [%d]\n", dccntr->soc2dev_ret_enq_dccntr_len);
	DC_DP_DBG("dev2soc_enq_phys_base      [0x%pK]\n", dccntr->dev2soc_enq_phys_base);
	DC_DP_DBG("dev2soc_enq_base           [0x%pK]\n", dccntr->dev2soc_enq_base);
	DC_DP_DBG("dev2soc_enq_dccntr_len     [%d]\n", dccntr->dev2soc_enq_dccntr_len);
	DC_DP_DBG("dev2soc_ret_deq_phys_base  [0x%pK]\n", dccntr->dev2soc_ret_deq_phys_base);
	DC_DP_DBG("dev2soc_ret_deq_base       [0x%pK]\n", dccntr->dev2soc_ret_deq_base);
	DC_DP_DBG("dev2soc_ret_deq_dccntr_len [%d]\n", dccntr->dev2soc_ret_deq_dccntr_len);
	DC_DP_DBG("flags                      [0x%x]\n", dccntr->flags);
}

static void inline dc_dp_dump_ring(struct dc_dp_ring *ring, char *name)
{
	DC_DP_DBG("name         %s\n", name);
	DC_DP_DBG("base         0x%pK\n", ring->base);
	DC_DP_DBG("phys_base    0x%pK\n", ring->phys_base);
	DC_DP_DBG("size         %d\n", ring->size);
	DC_DP_DBG("flags        0x%x\n", ring->flags);
	DC_DP_DBG("ring         0x%pK\n", ring->ring);
}

static void inline dc_dp_dump_resources(char *prefix, struct dc_dp_res *res)
{
	DC_DP_DBG("%s\n", prefix);
	DC_DP_DBG("num_bufs_req %d\n", res->num_bufs_req);
	DC_DP_DBG("num_bufpools %d\n", res->num_bufpools);
	DC_DP_DBG("buflist      0x%pK\n", res->buflist);
	DC_DP_DBG("tx_buflist      0x%pK\n", res->tx_buflist);
	DC_DP_DBG("num_dccntr   %d\n", res->num_dccntr);
	DC_DP_DBG("dev_priv_res 0x%pK\n", res->dev_priv_res);

	dc_dp_dump_ring(&res->rings.soc2dev, "soc2dev");
	dc_dp_dump_ring(&res->rings.soc2dev_ret, "soc2dev_ret");
	dc_dp_dump_ring(&res->rings.dev2soc, "dev2soc");
	dc_dp_dump_ring(&res->rings.dev2soc_ret, "dev2soc_ret");

	dc_dp_dump_dccntr(res->dccntr);
}
#endif

static void skb_dump(struct tc_priv *priv,
		struct sk_buff *skb, int len, int flag)
{
	int i;
	u32 *data;

	if (!(priv->msg_enable & flag))
		return;

	if (len > skb->len)
		len = skb->len;

	if (flag == MSG_RXDATA)
		printk("RX DATA from lib: len: %d\n", skb->len);
	if (flag == MSG_TXDATA)
		printk("TX DATA to lib: len: %d\n", skb->len);

	if (len % 4 != 0)
		len = len / 4 + 1;
	else
		len = len / 4;

	data = (u32 *)skb->data;
	for (i = 0; i < len; i++) {
		if (i % 4 == 0)
			printk("\n0x%08x: ", (u32)data + i * 4);
		printk("0x%08x  ", data[i]);
	}
	printk("\n");
}

static int plat_data_mmap(void *data, int len, enum dma_data_direction dir)
{
	dma_addr_t phyaddr;
	struct device *pdev = get_plat_device();

	phyaddr = dma_map_single(pdev, data, len, dir);
	if (unlikely(dma_mapping_error(pdev, phyaddr))) {
		WARN_ONCE(1, "DMA address mapping fail!: dir: %d, vaddr:0x%08x\n",
			dir, (u32)data);
		return -1;
	}
	dma_unmap_single(pdev, phyaddr, len, dir);
	return phyaddr;
}

static int32_t plat_rx(struct net_device *rxdev, struct net_device *txdev, struct dp_subif *rx_subif,
	struct sk_buff **skb, int32_t len, uint32_t flag)
{
	int32_t err;
	struct tc_priv *tcpriv = plat_to_tcpriv();
	struct sk_buff *skb_rx;

	if (unlikely(!rxdev)) {
		if (txdev != NULL)
			tc_dbg(tcpriv, MSG_RX,
				"Recv undelivered packet from DP lib\n");
		else
			tc_dbg(tcpriv, MSG_RX, "Recv unknown packet\n");
		err = -ENODEV;
		goto err1;
	}

	skb_rx = *skb;
	plat_data_mmap(skb_rx->data, skb_rx->len, DMA_FROM_DEVICE);

	skb_dump(tcpriv, skb_rx, skb_rx->len, MSG_RXDATA);
	skb_rx->DW0 = rx_subif->subif;

	tcpriv->tc_ops.recv(rxdev, skb_rx);
	*skb = NULL;

	return 0;

err1:
	dev_kfree_skb_any(*skb);

	return err;
}

static int32_t plat_get_subifid(struct net_device *netif,
	struct sk_buff *skb, void *subif_data, char *mac_addr,
	struct dp_subif *subif, uint32_t flags)
{
	int qid;
	struct tc_priv *priv = plat_to_tcpriv();

	qid = priv->tc_ops.get_qid(netif, skb, subif_data, 0);

	if(qid < 0)
		return qid;

	subif->subif = qid;

	return 0;
}

/* This API won't be called in ATM mode */
int32_t plat_get_desc_info(int32_t port_id, struct sk_buff *skb,
	struct dc_dp_fields_value_dw *desc_fields, uint32_t flags)
{
	int idx = 0;
	int qid;
	struct dc_dp_fields_dw *dw;
	struct tc_priv *priv = plat_to_tcpriv();

	desc_fields->num_dw = 1;
	dw = &desc_fields->dw[idx];

	qid = priv->tc_ops.get_qid(skb->dev, skb, NULL, flags);
	if (qid < 0)
		qid = 0;
	dw->dw = 0;
	dw->desc_val = qid;
	dw->desc_mask = 0x7F;

	return 0;
}

static int plat_send(struct net_device *pdev, struct sk_buff *skb,
	int qid, enum tc_pkt_type type)
{
	struct plat_priv *priv = g_plat_priv;
	struct dp_subif dp_id;
	u32 flags;

	dp_id.port_id = priv->port_id;
	dp_id.subif = qid;
	flags = 0;

	set_mask_bit(skb->DW1, priv->port_id, SKB_PORT_ID, SKB_PORT_ID_S);
	set_mask_bit(skb->DW0, qid, SKB_PORT_SUBID, SKB_PORT_SUBID_S);
#ifdef CONFIG_SOC_TYPE_XWAY
	if (type == PTM_BOND_PKT)
		flags = DP_TX_DSL_FCS | DP_TX_BOND;
	if (type ==ATM_OAM_PKT)
		flags = DP_TX_OAM;
	if (type == ATM_SL_PKT)
		flags |= DP_TX_ATM;
#else
	flags = 0;
#endif


	skb_dump(priv->tc_priv, skb, skb->len, MSG_TXDATA);

	plat_data_mmap(skb->data, skb->len, DMA_TO_DEVICE);
	return dc_dp_xmit(pdev, NULL, &dp_id, skb, skb->len, flags);
}

/* return virtual address */
static void *plat_mem_alloc(size_t size,
		enum tc_dir dir, u32 *phyaddr)
{
	struct dc_dp_res *res = &g_plat_priv->soc_res;
	void *ptr = NULL;

	if (dir == US_DIR) {
		if (res->tx_buflist) {
			ptr = res->tx_buflist->pool;
			if (phyaddr)
				*phyaddr = (u32)res->tx_buflist->phys_pool;
			res->tx_buflist++;
		}
	} else if (dir == DS_DIR) {
		if (res->buflist) {
			ptr = res->buflist->pool;
			if (phyaddr)
				*phyaddr = (u32)res->buflist->phys_pool;
			res->buflist++;
		}
	}
	if (!ptr) {
		struct plat_priv *priv = g_plat_priv;
		struct dp_subif dp_id;
		struct sk_buff *skb;
		dp_id.port_id = priv->port_id;
		dp_id.subif = -1;
		skb = dc_dp_alloc_skb(size, &dp_id, 0);
		if (skb)
			ptr = skb->data;
		else
			ptr = NULL;
	}
	if (ptr) {
		if ((*phyaddr = plat_data_mmap(ptr, size, DMA_FROM_DEVICE)) < 0) {
			pr_err("dir: %d, phy addr: 0x%x, vaddr: 0x%x\n",
				(unsigned int)dir, *phyaddr, (u32)ptr);
			*phyaddr = 0;
			ptr = NULL;
		}
	}
	return ptr;
}

static void plat_mem_free(dma_addr_t phy_addr, enum tc_dir dir)
{
	struct plat_priv *priv;
	struct dp_subif subif_id;

	priv = g_plat_priv;
	if (priv) {
		struct sk_buff *skb = dev_alloc_skb(32);
		unsigned int *ptr_data;
		/* save the dma physical address to newly created skb */
		ptr_data = (unsigned int *)skb->head;
		*ptr_data = phy_addr;
		subif_id.port_id = priv->port_id;
		subif_id.subif = -1;
		/* build skb */
		dc_dp_free_skb(&subif_id, skb);
	}
}

static int plat_soc_res_init(struct dc_dp_res *res, struct soc_cfg *rings,
	int ep_id, int is_bonding)
{
	struct dc_dp_dccntr *cnt;
	int endian;

	if (!res->num_dccntr)
		memset(res, 0, sizeof(struct dc_dp_res));

	if (!is_bonding || ep_id == US_BOND)
		res->tx_num_bufs_req = rings->txin.req_buf_num;
	if (!is_bonding || ep_id == DS_BOND)
		res->num_bufs_req = rings->rxin.req_buf_num;

	res->num_dccntr = 1;
	if (!is_bonding || ep_id == US_BOND) {
		res->rings.soc2dev.size = rings->txin.soc_dnum;
		res->rings.soc2dev_ret.size = rings->txout.soc_dnum;
	}
	if (!is_bonding || ep_id == DS_BOND) {
		res->rings.dev2soc.size = rings->rxout.soc_dnum;
		res->rings.dev2soc_ret.size = rings->rxin.soc_dnum;
	}
	if (res->dccntr == NULL)
		res->dccntr = kzalloc(sizeof(struct dc_dp_dccntr), GFP_KERNEL);
	if (!res->dccntr) {
		pr_err("Cannot allocate dc_dp_dccntr\n");
		return -ENOMEM;
	}
	cnt = res->dccntr;

	endian = (rings->txin.soc_endian == ACA_BIG_ENDIAN) ?
		DC_DP_F_DCCNTR_MODE_BIG_ENDIAN : DC_DP_F_DCCNTR_MODE_LITTLE_ENDIAN;

	cnt->soc_write_dccntr_mode = DC_DP_F_DCCNTR_MODE_INCREMENTAL | endian;
	if (!is_bonding || ep_id == US_BOND) {
		cnt->soc2dev_enq_phys_base = (void *)rings->txin.aca_cnt_phyaddr;
		cnt->soc2dev_enq_base = rings->txin.aca_cnt_addr;
		cnt->soc2dev_enq_dccntr_len = sizeof(unsigned int);

		cnt->soc2dev_ret_deq_phys_base = (void *)rings->txout.aca_cnt_phyaddr;
		cnt->soc2dev_ret_deq_base = rings->txout.aca_cnt_addr;
		cnt->soc2dev_ret_deq_dccntr_len = sizeof(unsigned int);
	}
	if (!is_bonding || ep_id == DS_BOND) {
		cnt->dev2soc_deq_phys_base = (void *)rings->rxout.aca_cnt_phyaddr;
		cnt->dev2soc_deq_base = rings->rxout.aca_cnt_addr;
		cnt->dev2soc_deq_dccntr_len = sizeof(unsigned int);

		cnt->dev2soc_ret_enq_phys_base = (void *)rings->rxin.aca_cnt_phyaddr;
		cnt->dev2soc_ret_enq_base = rings->rxin.aca_cnt_addr;
		cnt->dev2soc_ret_enq_dccntr_len = sizeof(unsigned int);
	}
	/* TODO: 08-Sep: check TX/RX resources */
#if 0
	printk("%s: %x %x\n", __func__,
		cnt->soc2dev_enq_phys_base, cnt->dev2soc_deq_phys_base);
#endif
	//if (!cnt->soc2dev_enq_phys_base || !cnt->dev2soc_deq_phys_base)
	//	return -ENOMEM;
	return 0;
}

/* driver guarantee ring size must be bigger than request buffer size */
#define def_ring_size	1024
static unsigned int ring_size_probe(struct ring_param *ring)
{
	int i;
	if (ring->req_buf_num < def_ring_size)
		return def_ring_size;

	for (i = 0; BIT(i) < ring->req_buf_num; i++)
		;

	return BIT(i);
}

static void soc_ring_init(struct soc_cfg *cfg, int ep_id, int is_bonding)
{
	if (!is_bonding || ep_id == US_BOND) {
		cfg->txin.soc_dnum = ring_size_probe(&cfg->txin);
		cfg->txout.soc_dnum = ring_size_probe(&cfg->txout);
	}
	if (!is_bonding || ep_id == US_BOND) {
		cfg->rxout.soc_dnum = ring_size_probe(&cfg->rxout);
		cfg->rxin.soc_dnum = ring_size_probe(&cfg->rxin);
	}
	if (cfg->rxout.soc_dnum < cfg->rxin.soc_dnum)
		cfg->rxout.soc_dnum = cfg->rxin.soc_dnum;
}

static int plat_dp_init(struct plat_priv *priv, const char *drv_name)
{
	struct tc_priv *tcpriv;
	struct soc_cfg *cfg;
	struct dc_dp_host_cap cap;
	unsigned int endian;
	int dp_id;
	int flags;

	tcpriv = priv->tc_priv;
	priv->owner = THIS_MODULE;
	if (!priv->owner) {
		memset(&tc_mod.name, 0, sizeof(tc_mod.name));
		snprintf(tc_mod.name, MODULE_NAME_LEN - 1, "%s", drv_name);
		priv->owner = &tc_mod;
	}

	flags = DC_DP_F_FASTPATH | DC_DP_F_LITEPATH | DC_DP_F_SWPATH;

	dp_id  = dc_dp_alloc_port(priv->owner, 0, NULL, 0, flags);
	if (dp_id < 0) {
		tc_err(tcpriv, MSG_INIT, "dp_alloc_port fail!\n");
		return -ENOMEM;
	}
	priv->port_id = dp_id;

	if (dc_dp_get_host_capability(&cap, 0) < 0)
		goto __cap_fail;

	/**
	 * One platform only support one endian mode,
	 * always take fastpath mode fow now
	 */
	cfg = &tcpriv->cfg;
	if (cap.fastpath.support) {
		endian = (cap.fastpath.hw_cmode.soc2dev_write
			& DC_DP_F_DCCNTR_MODE_LITTLE_ENDIAN)
			? ACA_LITTLE_ENDIAN : ACA_BIG_ENDIAN;
		cfg->tx_fcs_en = cap.fastpath.hw_cap & DC_DP_F_HOST_CAP_TX_FCS;
		cfg->rx_fcs_en = cap.fastpath.hw_cap & DC_DP_F_HOST_CAP_RX_FCS;
		cfg->intr_en = 0;
	} else {
		endian = (cap.swpath.sw_cmode.soc2dev_write
			& DC_DP_F_DCCNTR_MODE_LITTLE_ENDIAN)
			? ACA_LITTLE_ENDIAN : ACA_BIG_ENDIAN;
		cfg->tx_fcs_en = cap.fastpath.hw_cap & DC_DP_F_HOST_CAP_TX_FCS;
		cfg->rx_fcs_en = cap.fastpath.hw_cap & DC_DP_F_HOST_CAP_RX_FCS;
		cfg->intr_en = 1;
	}

	cfg->txin.soc_endian = endian;
	cfg->txout.soc_endian = endian;
	cfg->rxin.soc_endian = endian;
	cfg->rxout.soc_endian = endian;

	return 0;

__cap_fail:
	dc_dp_alloc_port(priv->owner, 0, NULL, priv->port_id, DC_DP_F_DEREGISTER);
	return -ENODEV;
}

static void plat_dp_exit(struct plat_priv *priv)
{
	dc_dp_alloc_port(priv->owner, 0, NULL, priv->port_id, DC_DP_F_DEREGISTER);
}

static int plat_soc_cfg_get(struct soc_cfg *cfg, u32 id)
{
	return 0;
}

static int plat_open(struct net_device *pdev, const char *dev_name,
		int *subif, int flag)
{
	struct plat_priv *priv;
	struct dp_subif subif_id;
	int ret;

	if (WARN_ON(pdev == NULL && dev_name == NULL))
		return -ENODEV;

	priv = g_plat_priv;
	subif_id.port_id = priv->port_id;
	subif_id.subif = -1;
	ret = dc_dp_register_subif(priv->owner, pdev, dev_name, &subif_id, 0);
	if (ret < 0) {
		tc_err(priv->tc_priv, MSG_INIT, "Register subif fail!\n");
		return ret;
	}
	*subif = subif_id.subif;

	return 0;
}

static void plat_close(struct net_device *pdev, const char *dev_name,
		int subif, int flag)
{
	struct plat_priv *priv;
	struct dp_subif subif_id;
	int ret;

	if (WARN_ON(pdev == NULL && dev_name == NULL))
		return;

	priv = g_plat_priv;
	subif_id.port_id = priv->port_id;
	subif_id.subif = subif;
	ret = dc_dp_register_subif(priv->owner, pdev, dev_name,
			&subif_id, DC_DP_F_DEREGISTER);
	if (ret < 0) {
		tc_err(priv->tc_priv, MSG_INIT,
			"Unregister pid(%d) subif(%d) fail!\n",
			priv->port_id, subif);
	}

	return;
}

static int plat_soc_res_check(struct tc_priv *priv,
		struct dc_dp_res *res, struct soc_cfg *cfg)
{
	struct ring_param *ring;
#ifndef CONFIG_SOC_TYPE_XWAY
	if (res->tx_num_bufs_req && !res->tx_buflist) {
		tc_err(priv, MSG_SWITCH, "txin buflist is NULL\n");
		goto __mem_fail;
	}
	if (res->num_bufs_req && !res->buflist) {
		tc_err(priv, MSG_SWITCH, "rxin buflist is NULL\n");
		goto __mem_fail;
	}

	if (res->tx_num_bufs_req > res->tx_num_bufpools) {
		tc_err(priv, MSG_SWITCH,
			"txin req buf num: %d, txin ret buf num: %d\n",
			res->tx_num_bufs_req, res->tx_num_bufpools);
		goto __mem_fail;
	}
	if (res->num_bufs_req > res->num_bufpools) {
		tc_err(priv, MSG_SWITCH,
			"rxin req buf num: %d, rxin ret buf num: %d\n",
			res->num_bufs_req, res->num_bufpools);
		goto __mem_fail;
	}
#endif
	ring = &cfg->txin;
	ring->soc_dbase = res->rings.soc2dev.base;
	ring->soc_phydbase = (unsigned int)res->rings.soc2dev.phys_base;
	ring->soc_dnum = res->rings.soc2dev.size;
	ring->soc_cnt_phyaddr = (unsigned int)res->dccntr->soc2dev_deq_phys_base; //0x50320000;
	ring->soc_desc_dwsz = res->rings.soc2dev.desc_dwsz;
#ifndef CONFIG_SOC_TYPE_XWAY
	ring->prefill_cnt = res->tx_num_bufpools; /* TODO: should from dcdp */
#else
	ring->prefill_cnt = 0;
#endif
	tc_dbg(priv, MSG_SWITCH,
		"txin: base: 0x%x, phybase: 0x%x, dnum: %d, soc cnt addr: 0x%x, dw_sz: %d\n",
		(u32)ring->soc_dbase, ring->soc_phydbase,
		ring->soc_dnum, ring->soc_cnt_phyaddr, ring->soc_desc_dwsz);

	ring = &cfg->txout;
	ring->soc_dbase = res->rings.soc2dev_ret.base;
	ring->soc_phydbase = (unsigned int)res->rings.soc2dev_ret.phys_base;
	ring->soc_dnum = res->rings.soc2dev_ret.size;
	ring->soc_cnt_phyaddr = (unsigned int)res->dccntr->soc2dev_ret_enq_phys_base;
	ring->soc_desc_dwsz = res->rings.soc2dev_ret.desc_dwsz;
	ring->prefill_cnt = 0;
	tc_dbg(priv, MSG_SWITCH,
		"txout: base: 0x%x, phybase: 0x%x, dnum: %d, soc cnt addr: 0x%x, dw_sz: %d\n",
		(u32)ring->soc_dbase, ring->soc_phydbase,
		ring->soc_dnum, ring->soc_cnt_phyaddr, ring->soc_desc_dwsz);

	ring = &cfg->rxin;
	ring->soc_dbase = res->rings.dev2soc_ret.base;
	ring->soc_phydbase = (unsigned int)res->rings.dev2soc_ret.phys_base;
	ring->soc_dnum = res->rings.dev2soc_ret.size;
	ring->soc_cnt_phyaddr = (unsigned int)res->dccntr->dev2soc_ret_deq_phys_base;
	ring->soc_desc_dwsz = res->rings.dev2soc_ret.desc_dwsz;
	/* ring->prefill_cnt = res->num_bufpools; *//* TODO: should from dcdp */
#ifndef CONFIG_SOC_TYPE_XWAY
	ring->prefill_cnt = res->num_bufs_req; /* TODO: should from dcdp */
#else
	ring->prefill_cnt = 0;
#endif
	tc_dbg(priv, MSG_SWITCH,
		"rxin: base: 0x%x, phybase: 0x%x, dnum: %d, soc cnt addr: 0x%x, dw_sz: %d\n",
		(u32)ring->soc_dbase, ring->soc_phydbase,
		ring->soc_dnum, ring->soc_cnt_phyaddr, ring->soc_desc_dwsz);

	ring = &cfg->rxout;
	ring->soc_dbase = res->rings.dev2soc.base;
	ring->soc_phydbase = (unsigned int)res->rings.dev2soc.phys_base;
	ring->soc_dnum = res->rings.dev2soc.size;
	ring->soc_cnt_phyaddr = (unsigned int)res->dccntr->dev2soc_enq_phys_base;
	ring->soc_desc_dwsz = res->rings.dev2soc.desc_dwsz;
	ring->prefill_cnt = 0;
	tc_dbg(priv, MSG_SWITCH,
		"rxout: base: 0x%x, phybase: 0x%x, dnum: %d, soc cnt addr: 0x%x, dw_sz: %d\n",
		(u32)ring->soc_dbase, ring->soc_phydbase,
		ring->soc_dnum, ring->soc_cnt_phyaddr, ring->soc_desc_dwsz);

	return 0;
#ifndef CONFIG_SOC_TYPE_XWAY
__mem_fail:
#endif
	return -ENOMEM;
}

static int plat_reg(struct net_device *pdev, const char *dev_name,
        int id, int flag)
{
	struct plat_priv *priv;
	struct soc_cfg *cfg;
	int ret;
	int dev_reg_flag;

	priv = g_plat_priv;
	priv->cb.stop_fn = NULL;
	priv->cb.restart_fn = NULL;
	priv->cb.rx_fn = plat_rx;
	priv->cb.get_subif_fn = plat_get_subifid;
	priv->cb.get_desc_info_fn =  plat_get_desc_info;

	cfg =  &priv->tc_priv->cfg;
	DC_DP_DBG("%s: id[%d]flag[%d]\n", __func__, id, flag);
	soc_ring_init(cfg, id, flag);
	ret = plat_soc_res_init(&priv->soc_res, cfg, id, flag);
	DC_DP_DBG("%s: ret[%d]\n", __func__, ret);
	if (ret)
		goto __res_alloc_fail;
		#ifdef CONFIG_X86
		dev_reg_flag = 0;
		#else
	if (flag && id == DS_BOND)
	#ifdef DC_DP_F_SHARED_RES
		dev_reg_flag = DC_DP_F_SHARED_RES;
	#else
		dev_reg_flag = DC_DP_F_UPDATE;
	#endif
	else
		dev_reg_flag = 0;
		#endif
	ret = dc_dp_register_dev(priv->owner, priv->port_id, pdev,
			&priv->cb, &priv->soc_res, &priv->dev, dev_reg_flag);
	dc_dp_dump_resources("After DEV REG: ", &priv->soc_res);
	//if (ret)
		//goto __reg_fail;

	/* Check output of registration */
	ret = plat_soc_res_check(priv->tc_priv, &priv->soc_res, cfg);
	if (ret)
		goto __check_fail;
	return 0;

__check_fail:
	dc_dp_register_dev(priv->owner, priv->port_id, pdev, &priv->cb,
		&priv->soc_res, &priv->dev, DC_DP_F_DEREGISTER);
//__reg_fail:
	kfree(priv->soc_res.dccntr);
__res_alloc_fail:
	return ret;
}

static void plat_unreg(struct net_device *pdev, const char *dev_name, int flag)
{
	struct plat_priv *priv = g_plat_priv;

	dc_dp_register_dev(priv->owner, priv->port_id, pdev, &priv->cb,
		&priv->soc_res, &priv->dev, DC_DP_F_DEREGISTER);
	if (priv->soc_res.dccntr)
		kfree(priv->soc_res.dccntr);
	memset(&priv->soc_res, 0, sizeof(struct dc_dp_res));
}

static int plat_get_mib(struct net_device *pdev,
			struct rtnl_link_stats64 *stat)
{
	return -ENOTSUPP;
}

static void plat_tc_ops_setup(struct tc_priv *priv)
{
	priv->tc_ops.send = plat_send;
	priv->tc_ops.alloc = plat_mem_alloc;
	priv->tc_ops.free = plat_mem_free;
	priv->tc_ops.subif_reg = plat_open;
	priv->tc_ops.subif_unreg = plat_close;
	priv->tc_ops.dev_reg = plat_reg;
	priv->tc_ops.dev_unreg = plat_unreg;
	priv->tc_ops.umt_init = NULL;
	priv->tc_ops.umt_exit = NULL;
	priv->tc_ops.umt_start = NULL;
	priv->tc_ops.soc_cfg_get = plat_soc_cfg_get;
	priv->tc_ops.coc_req = NULL;
	priv->tc_ops.disable_us = NULL;
	priv->tc_ops.get_mib = plat_get_mib;
}

static int showtime_stat(struct tc_priv *priv)
{
	int i;

	for (i = 0; i < priv->ep_num; i++) {
		if (priv->showtime[i] == 1)
			return 1;
	}

	return 0;
}

static int showtime_enter(const unsigned char idx,
		struct port_cell_info *cell_info, void *data)
{
	struct tc_priv *priv;
	u32 status;

	if (WARN_ON(idx >= EP_MAX_NUM || cell_info == NULL))
		return -EINVAL;

	priv = g_plat_priv->tc_priv;

	status = showtime_stat(priv);
	priv->showtime[idx] = 1;

	spin_lock_bh(&priv->tc_lock);
	if (priv->tc_stat == TC_RUN && priv->tc_ops.showtime_enter != NULL)
		priv->tc_ops.showtime_enter(idx, cell_info, data);
	spin_unlock_bh(&priv->tc_lock);

	return 0;
}

static int showtime_exit(const unsigned char idx)
{
	struct tc_priv *priv;

	if (WARN_ON(idx >= EP_MAX_NUM))
		return -EINVAL;

	priv = g_plat_priv->tc_priv;

	priv->showtime[idx] = 0;

	/* if (showtime_stat(priv))
		return 0; */

	spin_lock_bh(&priv->tc_lock);
	if (priv->tc_stat == TC_RUN && priv->tc_ops.showtime_exit != NULL)
		priv->tc_ops.showtime_exit(idx);
	spin_unlock_bh(&priv->tc_lock);

	return 0;
}

static int plat_erb_addr_get(const unsigned char idx,
	unsigned int *data_addr, unsigned int *desc_addr)
{
	struct tc_priv *priv;

	if (WARN_ON(idx >= EP_MAX_NUM || !data_addr || !desc_addr))
		return -EINVAL;

	priv = g_plat_priv->tc_priv;

	if (priv->tc_stat == TC_RUN && priv->tc_ops.erb_addr_get != NULL)
		return priv->tc_ops.erb_addr_get(idx, data_addr, desc_addr);
	else {
		/* MEI driver request both value set to zero in error condition */
		*data_addr = *desc_addr = 0;
		tc_err(priv, MSG_EVENT,
			"erb_addr_get not supported or timing not correct!, tc stat: %u\n",
			(u32)priv->tc_stat);
		return -1;
	}

	return -1;
}

static void plat_tc_req_workqueue(struct work_struct *work)
{
	struct tc_req *req_work;

	req_work = container_of(work, struct tc_req, work);
	tc_request(req_work->id, req_work->tc_mode);
}

static int plat_tc_request(const unsigned char id,
		mei_tc_request_type tc_type, int is_bonding)
{
	enum dsl_tc_mode mode;
	struct tc_priv *priv;
	int i;

	if (WARN_ON(id >= EP_MAX_NUM))
		return -EINVAL;

	priv = g_plat_priv->tc_priv;

	switch (tc_type) {
	case MEI_TC_REQUEST_OFF:
		mode = TC_NONE_MODE;
		for (i = 0; i < priv->ep_num; i++) {
			if (priv->showtime[i] == 1)
				showtime_exit(i);
		}
		break;
	case MEI_TC_REQUEST_ATM:
		mode = TC_ATM_SL_MODE;
		break;
	case MEI_TC_REQUEST_PTM:
		if (is_bonding)
			mode = TC_PTM_BND_MODE;
		else
			mode = TC_PTM_SL_MODE;
		break;
	default:
		return -EINVAL;
	}

	tc_dbg(priv, MSG_EVENT,
		"%s: dsl id: %d, mode: %d, tc_mode: %d, tc_idx: %d\n",
		__func__, id, mode, priv->tc_mode, priv->tc_idx);

	if (mode == priv->tc_mode) {
		if (((((mode == TC_PTM_SL_MODE) || (mode == TC_ATM_SL_MODE))
			&& (id == priv->tc_idx)) || (mode == TC_PTM_BND_MODE))
			&& (priv->tc_ops.framer_request_en != NULL)) {
			priv->tc_ops.framer_request_en(id);
			return 0;
		}
	}

	spin_lock(&priv->tc_lock);
	if (priv->tc_stat != TC_RUN && priv->tc_stat != NO_TC) {
		tc_err(priv, MSG_SWITCH,
			"TC status(%d) not allowed to switch\n", priv->tc_stat);
		spin_unlock(&priv->tc_lock);
		return -1;
	}
	priv->tc_stat = TC_SWITCHING;
	spin_unlock(&priv->tc_lock);

	g_plat_priv->req_work.id = id;
	g_plat_priv->req_work.tc_mode = mode;
	queue_work(system_long_wq, &g_plat_priv->req_work.work);

	return 0;
}

void simu_tc_request(const u8 id, int type)
{
	switch (type) {
	case TC_NONE_MODE:
		plat_tc_request(id, MEI_TC_REQUEST_OFF, 0);
		break;
	case TC_ATM_SL_MODE:
		plat_tc_request(id, MEI_TC_REQUEST_ATM, 0);
		break;
	case TC_PTM_SL_MODE:
		plat_tc_request(id, MEI_TC_REQUEST_PTM, 0);
		break;
	case TC_PTM_BND_MODE:
		plat_tc_request(id, MEI_TC_REQUEST_PTM, 1);
		break;
	}
}

static inline void plat_dsl_ops_setup(void)
{
	ppa_callback_set(LTQ_MEI_SHOWTIME_ENTER, showtime_enter);
	ppa_callback_set(LTQ_MEI_SHOWTIME_EXIT, showtime_exit);
	ppa_callback_set(LTQ_MEI_TC_REQUEST, plat_tc_request);
	ppa_callback_set(LTQ_MEI_ERB_ADDR_GET, plat_erb_addr_get);
}

static inline void plat_dsl_ops_exit(void)
{
	ppa_callback_set(LTQ_MEI_SHOWTIME_ENTER, NULL);
	ppa_callback_set(LTQ_MEI_SHOWTIME_EXIT, NULL);
	ppa_callback_set(LTQ_MEI_TC_REQUEST, NULL);
	ppa_callback_set(LTQ_MEI_ERB_ADDR_GET, NULL);
}

int platform_init(struct tc_priv *tc_priv, const char *drv_name)
{
	struct plat_priv *priv;
	int ret;

	priv = (struct plat_priv *)tc_plat_priv(tc_priv);
	priv->tc_priv = tc_priv;
	g_plat_priv = priv;

	ret = plat_dp_init(priv, drv_name);
	if (ret < 0)
		goto err1;

	INIT_WORK(&priv->req_work.work, plat_tc_req_workqueue);
	plat_tc_ops_setup(tc_priv);
	plat_dsl_ops_setup();

	return 0;

err1:
	g_plat_priv = NULL;
	return ret;
}

void platform_dsl_exit(void)
{
	plat_dsl_ops_exit();
	cancel_work_sync(&g_plat_priv->req_work.work);
}

void platform_exit(void)
{
	plat_dp_exit(g_plat_priv);
	g_plat_priv = NULL;
}
