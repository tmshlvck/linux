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

#include "../inc/dsl_tc.h"
#include "../inc/tc_main.h"
#include "../inc/reg_addr.h"
#include "../inc/tc_common.h"


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

enum {
	RXIN_DIR = 0,
	TXIN_DIR,
	RXOUT_DIR,
	TXOUT_DIR,
};

/* SoC Ring size */
#define TXIN_DNUM  	128
#define TXOUT_DNUM	128
#define RXOUT_DNUM	1024
#define RXIN_DNUM	1024

#define TXIN_CHECK_NUM	32

struct aca_ring {
	void *dbase_mem;
	u32 dbase_phymem;
	void *umt_dst;
	u32 umt_phydst;
	u32 dnum;
	int idx; /* SoC RX/TX index */
	int cnt;
	void *cnt_addr;
	u32 cnt_phyaddr;
	int ep_dev_idx;
};

struct aca_ring_grp {
	struct aca_ring rxin;
	struct aca_ring txin;
	struct aca_ring rxout;
	struct aca_ring txout;
};

#if 1
struct dma_desc {
	/* DW 0 */
	u32 qid;
	/* DW 1 */
	u32 res2;
	/* DW 2 */
	u32 data_ptr;
	/* DW 3 */
	u32 data_len:16;
	u32 res0:7;
	u32 byte_off:3;
	u32 res1:2;
	u32 eop:1;
	u32 sop:1;
	u32 c:1;
	u32 own:1;
}__packed;
#else
struct dma_desc {
	/* DW 0 */
	u32 qid;
	/* DW 1 */
	u32 res;
	/* DW 2 */
	u32 data_ptr;
	/* DW 3 */
	u32 own:1;
	u32 c:1;
	u32 sop:1;
	u32 eop:1;
	u32 res1:2;
	u32 byte_off:3;
	u32 res0:7;
	u32 data_len:16;
}__packed;

#endif

struct plat_dma {
	u32 chan; /* CHAN IID */
	u32 dma_chan; /* CONTROLLER/PORT/CHAN ID */
	u32 ds_dnum; /* DS descriptor number */
};

struct plat_umt {
	u32 id;
	u32 cbm_id;
	u32 period;
	u32 dst;
};

struct tc_req {
	struct work_struct work;
	u32 id;
	enum dsl_tc_mode tc_mode;
};

#if 0
struct tc_coc {
	enum ltq_cpufreq_state coc_stat;
	struct tasklet_struct coc_task;
};
#endif

struct plat_priv {
	struct tc_priv *tc_priv;
	struct plat_umt umt[EP_MAX_NUM];
	struct plat_dma dma[EP_MAX_NUM];
	struct ltq_mei_atm_showtime_info dsl_ops;
	struct tc_req req_work;
	struct aca_ring_grp soc_rings;
	/* struct tc_coc coc;*/
};

static struct plat_priv *g_plat_priv;
struct tasklet_struct txout_task;
struct tasklet_struct rxout_task;

static void txout_action(struct tc_priv *priv, struct aca_ring *txout);

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

static int32_t plat_rx(struct net_device *rxdev, struct net_device *txdev,
	struct sk_buff *skb, int32_t len)
{
	int32_t err;
	struct tc_priv *tc_priv = plat_to_tcpriv();

	if (unlikely(!rxdev)) {
		if (txdev != NULL)
			tc_dbg(tc_priv, MSG_RX,
				"Recv undelivered packet from DP lib\n");
		else
			tc_dbg(tc_priv, MSG_RX, "Recv unknown packet\n");
		err = -ENODEV;
		goto err1;
	}

	tc_priv->tc_ops.recv(rxdev, skb);
	return 0;

err1:
	dev_kfree_skb_any(skb);

	return err;
}

#if 0
static int32_t plat_get_subifid(struct net_device *dev, struct sk_buff *skb,
	void *subif_data, uint8_t dst_mac[MAX_ETH_ALEN],
	dp_subif_t *subif, uint32_t flags)
{
	int qid;
	struct tc_priv *priv = plat_to_tcpriv();

	qid = priv->tc_ops.get_qid(dev, skb, subif_data, flags);
	if (qid < 0)
		return qid;
	else
		subif->subif = qid;

	return 0;
}
#endif

#if 0
static void plat_coc_tasklet(unsigned long arg)
{
	/* change state to D0 */
	if (g_plat_priv->coc.coc_stat == LTQ_CPUFREQ_PS_D0)
		return;

	g_plat_priv->coc.coc_stat = LTQ_CPUFREQ_PS_D0;
	/* call datapath to inform about the new state */
#if IS_ENABLED(CONFIG_LTQ_DATAPATH_CPUFREQ)
	dp_coc_new_stat_req(LTQ_CPUFREQ_PS_D0, DP_COC_REQ_VRX318);
#endif
}

static void plat_coc_req(void)
{
	tasklet_schedule(&g_plat_priv->coc.coc_task);
}


static int32_t plat_coc_stat(enum ltq_cpufreq_state new_state,
	enum ltq_cpufreq_state old_state, uint32_t flags)
{
	struct tc_priv *priv = plat_to_tcpriv();
	tc_dbg(priv, MSG_COC,
		"COC current state: %d, new state: %d, old state: %d\n",
		g_plat_priv->coc.coc_stat, new_state, old_state);

	if (g_plat_priv->coc.coc_stat != new_state) {
		g_plat_priv->coc.coc_stat = new_state;

		if (new_state == LTQ_CPUFREQ_PS_D3) {
			/* Enable interrupt for DS packet */
			priv->tc_ops.irq_on(MBOX_PKT_RX);
		} else {
			/* Disable interrupt for DS packet */
			priv->tc_ops.irq_off(MBOX_PKT_RX);
		}
	}

	return 0;
}
#endif

static inline int ring_dist(int idx1, int idx2, int size)
{
	if (idx1 >= idx2)
		return (idx1 - idx2);
	else
		return (idx1 + size - idx2);
}

static inline int __ring_full(int idx, int cnt, u32 dnum)
{
	if (ring_dist(idx, cnt, dnum) < dnum - 1)
		return 0;
	else
		return 1;
}

static inline int ring_full(struct aca_ring *ring)
{
	if (!__ring_full(ring->idx, ring->cnt, ring->dnum))
		return 0;

	/* if ring full, update cumulative counter and check again */
	ring->cnt = readl(ring->cnt_addr) % ring->dnum;

	return __ring_full(ring->idx, ring->cnt, ring->dnum);
}

#define ring_idx_inc(ring, idx)						\
	do { ring->idx = (ring->idx + 1) % ring->dnum; } while (0);

static inline void ring_cnt_update(struct aca_ring *ring)
{
	ring->cnt = readl(ring->cnt_addr) % ring->dnum;
}

static struct sk_buff *txin_skb_prepare(struct sk_buff *skb)
{
	struct sk_buff *nskb;

	/* Remove skb sharing */
	nskb= skb_share_check(skb, GFP_ATOMIC);

	/* Remove skb clone */
	if (skb_unclone(nskb, GFP_ATOMIC)) {
		pr_err("%s: unclone failed!\n", __func__);
		return NULL;
	}

	/* linear skb if necessary */
	if (skb_linearize(nskb)) {
		pr_err("%s: linear skb failed!\n", __func__);
		return NULL;
	}

	return nskb;
}

static int ring_mmap(void *mem, int size,
	enum dma_data_direction dir, u32 *addr)
{
	struct device *pdev;
	dma_addr_t phy_addr;
	struct tc_priv *priv;
	u32 addr1;

	priv = g_plat_priv->tc_priv;
	pdev = priv->ep_dev[0].dev;

	phy_addr = dma_map_single(pdev, mem, size, dir);
	if (unlikely(dma_mapping_error(pdev, phy_addr))) {
		tc_err(priv, MSG_INIT,
			"DMA address mapping error: buf: 0x%x, size: %d, dir: %d\n",
			(u32)mem, size, dir);
		return -ENOMEM;
	}
	dma_unmap_single(pdev, phy_addr, size, dir);

	pr_info("vaddr: 0x%x, phyaddr: 0x%lx\n", (u32)mem, phy_addr);
	addr1 = (u32)phy_addr;

	if (addr)
		*addr = addr1;

	return 0;
}

static void txin_action(struct tc_priv *priv, struct aca_ring *txin,
		struct sk_buff *skb, int qid, enum tc_pkt_type type)
{
	struct dma_desc *desc, desc1;
	u32 phyaddr, *dst, *src;
	int i;

	if (ring_full(txin)) {
		tc_dbg(priv, MSG_TX,
			"TXIN Ring Full!: idx: %d, cnt: %d\n",
			txin->idx, txin->cnt);
		goto err1;
	}

	skb = txin_skb_prepare(skb);
	if (!skb)
		return;

	if (ring_mmap(skb->data, skb->len, DMA_TO_DEVICE, &phyaddr) < 0) {
		tc_err(priv, MSG_TX, "TXIN data mmap failed: 0x%x\n",
			(unsigned int)skb->data);
		goto err1;
	}

	/* init a new descriptor for the new skb */
	desc = (struct dma_desc *)txin->dbase_mem;
	desc += txin->idx;

	memset(desc, 0, sizeof(*desc));
	memset(&desc1, 0, sizeof(desc1));
	desc1.own = 1;
	desc1.c = 1;
	desc1.sop = 1;
	desc1.eop = 1;
	desc1.byte_off = phyaddr & 0x7;
	desc1.data_len = skb->len;

	desc1.data_ptr = phyaddr & (~(0x7));
	desc1.qid = qid;

	dst = (u32 *)desc;
	src = (u32 *)&desc1;
	for (i = 0; i < DW_SZ(desc1); i++)
		dst[i] = cpu_to_be32(src[i]);

	pr_info("txin idx: %d\n", txin->idx);
	pr_info("descriptor dst val:(DW0-DW3): 0x%x, 0x%x, 0x%x, 0x%x\n",
		dst[0], dst[1], dst[2], dst[3]);
	pr_info("descriptor src val: (DW0-DW3): 0x%x, 0x%x, 0x%x, 0x%x\n",
		src[0], src[1], src[2], src[3]);

	if (ring_mmap(desc, sizeof(*desc), DMA_TO_DEVICE, NULL) < 0) {
		tc_err(priv, MSG_TX, "TXIN descriptor mmap failed: 0x%x\n",
			(unsigned int)desc);
		goto err1;
	}

	ring_idx_inc(txin, idx);

	/* update TXIN UMT by 1 */
	writel(1, txin->umt_dst);
	pr_info("TXIN send txin packet 1 packet\n");

	/* Free skb */
	//dev_kfree_skb_any(skb);

	/* check txout for testing*/
	//txout_action(plat_to_tcpriv(), &g_plat_priv->soc_rings.txout);
	return;

err1:
	//skb->delay_free = 0;
	dev_kfree_skb_any(skb);
}

static void txout_action(struct tc_priv *priv, struct aca_ring *txout)
{
	int i, cnt;
	struct dma_desc *desc;
	u32 ptr;
	void *mem;

	ring_cnt_update(txout);
	cnt = ring_dist(txout->idx, txout->cnt, txout->dnum);

	for (i = 0; i < cnt; i++) {
		desc = txout->dbase_mem;
		desc += txout->idx;
		/* read from memory */
		if (ring_mmap(desc, sizeof(*desc), DMA_FROM_DEVICE, NULL) < 0) {
			tc_err(priv, MSG_TX,
				"map TXOUT DMA descriptor failed\n");
			continue;
		}
		ptr = desc->data_ptr + desc->byte_off;
		mem = (void * __force)__va(ptr);
		kfree(mem);
		ring_idx_inc(txout, idx);
	}

	if (cnt)
		writel(cnt, txout->umt_dst);
	pr_info("TXOUT received %d descriptors\n", cnt);
}

static void rxin_action(struct tc_priv *priv,
		struct aca_ring *rxin, int size, int cnt)
{
	int i, dist;
	struct dma_desc *desc;
	void *data_ptr;
	u32 phyaddr;

	if (ring_full(rxin)) {
		tc_dbg(priv, MSG_RX,
			"RXIN Ring Full!: idx: %d, cnt: %d\n",
			rxin->idx, rxin->cnt);
		return;
	}

	dist = ring_dist(rxin->idx, rxin->cnt, rxin->dnum);
	if (cnt > dist) {
		WARN_ONCE(1, "RXIN NO enough room for free buffers: free: %d, room: %d\n",
			cnt, dist);
		cnt = dist;
	}

	for (i = 0; i < cnt; i++) {
		data_ptr = kmalloc(size, GFP_ATOMIC);
		if (!data_ptr) {
			tc_err(priv, MSG_RX,
				"RXIN kmalloc data buffer failed: %d\n", size);
			goto err1;
		}

		if (ring_mmap(data_ptr, size, DMA_FROM_DEVICE, &phyaddr) < 0) {
			tc_err(priv, MSG_RX,
				"RXIN kmalloc data buffer failed: %d\n", size);
			goto err2;
		}

		desc = (struct dma_desc *)rxin->dbase_mem;
		desc += rxin->idx;
		memset(desc, 0, sizeof(*desc));

		desc->data_len = size;
		desc->byte_off = phyaddr & 0x7;
		desc->eop = 1;
		desc->sop = 1;
		desc->own = 1;

		desc->data_ptr = phyaddr;


		if (ring_mmap(desc, sizeof(*desc), DMA_TO_DEVICE, NULL) < 0) {
			tc_err(priv, MSG_RX, "RXIN descriptor mmap failed: 0x%x\n",
				(unsigned int)desc);
			goto err2;
		}

		ring_idx_inc(rxin, idx);
	}

	/* update RXIN UMT*/
	writel(i, rxin->umt_dst);
	pr_info("rxin refill %d descriptors\n", i);
	return;

err2:
	kfree(data_ptr);
err1:
	if (i)
		writel(i, rxin->umt_dst);
	return;
}

static int rxout_action(struct tc_priv *priv, struct aca_ring *rxout)
{
	int i, cnt;
	struct dma_desc *desc;
	u32 ptr;
	void *mem;
	struct sk_buff *skb;

	ring_cnt_update(rxout);
	cnt = ring_dist(rxout->idx, rxout->cnt, rxout->dnum);

	for (i = 0; i < cnt; i++) {
		desc = rxout->dbase_mem;
		desc += rxout->idx;

		/* read from memory */
		if (ring_mmap(desc, sizeof(*desc), DMA_FROM_DEVICE, NULL) < 0) {
			tc_err(priv, MSG_RX,
				"map RXOUT DMA descriptor failed\n");
			continue;
		}
		ptr = desc->data_ptr + desc->byte_off;
		mem = __va(ptr);
		skb = build_skb(mem, 0);
		if (!skb) {
			tc_err(priv, MSG_RX,
				"RXOUT build skb failed\n");
			kfree(mem);
			continue;
		}
		priv->tc_ops.recv(NULL, skb);
		ring_idx_inc(rxout, idx);
	}

	if (!cnt)
		tc_err(priv, MSG_RX, "RXOUT dummy interrupt: dbase: 0x%x, idx: %d, cnt: %d\n",
			(unsigned int)rxout->dbase_mem, rxout->idx, rxout->cnt);
	else
		writel(cnt, rxout->umt_dst);

	pr_info("txout received %d packets\n", cnt);
	return cnt;
}

static void plat_txout_tasklet(unsigned long arg)
{
	struct plat_priv *priv = g_plat_priv;
	struct tc_priv *tcpriv = plat_to_tcpriv();
	struct aca_ring *txout = &priv->soc_rings.txout;
	struct dc_ep_dev *ep_dev = &tcpriv->ep_dev[txout->ep_dev_idx];

	txout_action(tcpriv, txout);

	/* Enable interrupt */
	ep_dev->hw_ops->icu_en(ep_dev, ACA_HOSTIF_TX);
}

static void plat_rxout_tasklet(unsigned long arg)
{
	struct plat_priv *priv = g_plat_priv;
	struct tc_priv *tcpriv = plat_to_tcpriv();
	struct aca_ring *rxout = &priv->soc_rings.rxout;
	struct aca_ring *rxin = &priv->soc_rings.rxin;
	struct dc_ep_dev *ep_dev = &tcpriv->ep_dev[rxout->ep_dev_idx];
	int cnt;


	cnt = rxout_action(tcpriv, rxout);
	if (cnt)
		rxin_action(tcpriv, rxin, DMA_PACKET_SZ, cnt);

	/* Enable interrupt */
	ep_dev->hw_ops->icu_en(ep_dev, ACA_HOSTIF_RX);
}

/**
 * send to ACA TXIN, UMT TXIN triggered by demand
 */
static int plat_send(struct net_device *pdev, struct sk_buff *skb,
	int qid, enum tc_pkt_type type)
{
	struct plat_priv *priv = g_plat_priv;
	struct aca_ring *txin = &priv->soc_rings.txin;

	txin_action(priv->tc_priv, txin, skb, qid, type);

	return 0;
}

/* return virtual address */
static void *plat_mem_alloc(size_t size, enum tc_dir dir, u32 *phyaddr)
{
	return kmalloc(size, GFP_KERNEL);
}

static void plat_mem_free(dma_addr_t phy_addr, enum tc_dir dir)
{
	void *mem;

	mem = (void * __force)__va(phy_addr);
	kfree(mem);
}

static void aca_soc_ring_init(struct tc_priv *priv,
			struct aca_ring *ring, u32 dnum, u32 dsize)
{
	int size;
	struct device *pdev;

	memset(ring, 0, sizeof(*ring));
	ring->dnum = dnum;
	size = dsize * dnum;
	pdev = priv->ep_dev[0].dev;

	ring->dbase_mem = kmalloc(size, GFP_KERNEL);
	if (!ring->dbase_mem) {
		tc_err(priv, MSG_INIT, "Allocate SoC Ring fail: %d\n", dnum);
		return;
	}

	ring_mmap(ring->dbase_mem, size, DMA_FROM_DEVICE, &(ring->dbase_phymem));
	tc_dbg(priv, MSG_INIT, "ring: membase: 0x%x, phybase: 0x%x, dnum: %d\n",
		(u32)ring->dbase_mem, ring->dbase_phymem, ring->dnum);

	size = sizeof(u32);
	ring->cnt_addr = kzalloc(size, GFP_KERNEL);
	if (!ring->cnt_addr) {
		tc_err(priv, MSG_INIT, "Allocate cumulative counter fail!\n");
		return;
	}

	ring_mmap(ring->cnt_addr, size, DMA_TO_DEVICE, &(ring->cnt_phyaddr));
	tc_dbg(priv, MSG_INIT, "ring: cumulative cnt addr: 0x%x, phy address: 0x%x\n",
		(u32)ring->cnt_addr, ring->cnt_phyaddr);

	return;
}

#define ring_init(tcpriv, ring, name1, name2, num, size)	\
{								\
	if (!tcpriv->param.name1##_dnum)			\
		num = name2##_DNUM;				\
	else							\
		num = tcpriv->param.name1##_dnum;		\
	aca_soc_ring_init(tcpriv, ring, num, size);		\
}

static irqreturn_t aca_rx_irq_handler(int irq, void *dev_id)
{
	struct dc_ep_dev *ep_dev = dev_id;

	/* Disable IRQ in IMCU */
	ep_dev->hw_ops->icu_mask(ep_dev, ACA_HOSTIF_RX);

	/* Start tasklet */
	tasklet_schedule(&rxout_task);

	return IRQ_HANDLED;
}

static irqreturn_t aca_tx_irq_handler(int irq, void *dev_id)
{
	struct dc_ep_dev *ep_dev = dev_id;

	/* Disable IRQ in IMCU */
	ep_dev->hw_ops->icu_mask(ep_dev, ACA_HOSTIF_TX);

	/* Start tasklet */
	tasklet_schedule(&txout_task);

	return IRQ_HANDLED;
}

static void irq_init(struct tc_priv *priv, const char *dev_name)
{
	int ret;
	int i;
	char name[IFNAMSIZ];

	for (i = 0; i < EP_MAX_NUM && i < priv->ep_num; i++) {
		sprintf(name, "%s%d", dev_name, i);

		ret = devm_request_irq(priv->ep_dev[i].dev, priv->ep_dev[i].aca_rx_irq,
				aca_rx_irq_handler, 0, name, &priv->ep_dev[i]);

		if (ret) {
			tc_err(priv, MSG_INIT,
				"ACA RX IRQ request Fail!: irq: %d, ep_id: %d\n",
				priv->ep_dev[i].aca_rx_irq, i);
			//return;
		}

		ret = devm_request_irq(priv->ep_dev[i].dev, priv->ep_dev[i].aca_tx_irq,
				aca_tx_irq_handler, 0, name, &priv->ep_dev[i]);

		if (ret) {
			tc_err(priv, MSG_INIT,
				"ACA TX IRQ request Fail!: irq: %d, ep_id: %d\n",
				priv->ep_dev[i].aca_tx_irq, i);
			//return;
		}
	}

	return;
}

/**
 * Decide txin/rxout queue size
 * Create a tx/rx queue
 */
static int plat_dp_init(struct plat_priv *priv, const char *drv_name)
{
	struct tc_priv *tcpriv;
	struct aca_ring_grp *soc_rings;
	struct aca_ring *ring;
	int size;
	u32 dnum;

	tcpriv = priv->tc_priv;

	size = sizeof(struct dma_desc);
	soc_rings = &priv->soc_rings;

	/* txin ring */
	ring = &soc_rings->txin;
	ring_init(tcpriv, ring, txin, TXIN, dnum, size);

	/* txout ring */
	ring = &soc_rings->txout;
	ring_init(tcpriv, ring, txout, TXOUT, dnum, size);
	/* rxin ring */
	ring = &soc_rings->rxin;
	ring_init(tcpriv, ring, rxin, RXIN, dnum, size);
	/* rxout ring */
	ring = &soc_rings->rxout;
	ring_init(tcpriv, ring, rxout, RXOUT, dnum, size);

	return 0;
}

/**
 *  disable TXOUT/RXOUT IRQ
 *  free txin/rxout descriptor ring
 */
static void plat_dp_exit(struct plat_priv *priv)
{
	return;
}

static int plat_soc_cfg_get(struct soc_cfg *cfg, u32 id)
{
	struct plat_priv *priv = g_plat_priv;

	/* TXIN */
	cfg->txin.soc_dbase = priv->soc_rings.txin.dbase_phymem;
	cfg->txin.soc_dnum = priv->soc_rings.txin.dnum;
	cfg->txin.soc_desc_dwsz = DW_SZ(struct dma_desc);
	cfg->txin.soc_cnt_phyaddr = priv->soc_rings.txin.cnt_phyaddr;
	/* TXOUT */
	cfg->txout.soc_dbase = priv->soc_rings.txout.dbase_phymem;
	cfg->txout.soc_dnum = priv->soc_rings.txout.dnum;
	cfg->txout.soc_desc_dwsz = DW_SZ(struct dma_desc);
	cfg->txout.soc_cnt_phyaddr = priv->soc_rings.txout.cnt_phyaddr;
	/* RXOUT */
	cfg->rxout.soc_dbase = priv->soc_rings.rxout.dbase_phymem;
	cfg->rxout.soc_dnum = priv->soc_rings.rxout.dnum;
	cfg->rxout.soc_desc_dwsz = DW_SZ(struct dma_desc);
	cfg->rxout.soc_cnt_phyaddr = priv->soc_rings.rxout.cnt_phyaddr;
	/* RXIN */
 	cfg->rxin.soc_dbase = priv->soc_rings.rxin.dbase_phymem;
	cfg->rxin.soc_dnum = priv->soc_rings.rxin.dnum;
	cfg->rxin.soc_desc_dwsz = DW_SZ(struct dma_desc);
	cfg->rxin.soc_cnt_phyaddr = priv->soc_rings.rxin.cnt_phyaddr;

	tc_info(priv->tc_priv, MSG_INIT,
		"id: %d, txin(0x%x: %d, 0x%x), txout(0x%x: %d, 0x%x), rxin(0x%x: %d, 0x%x), rxout(0x%x: %d, 0x%x)\n",
		id, cfg->txin.soc_dbase, cfg->txin.soc_dnum, cfg->txin.soc_cnt_phyaddr,
		cfg->txout.soc_dbase, cfg->txout.soc_dnum, cfg->txout.soc_cnt_phyaddr,
		cfg->rxin.soc_dbase, cfg->rxout.soc_dnum, cfg->rxin.soc_cnt_phyaddr,
		cfg->rxout.soc_dbase, cfg->rxout.soc_dnum, cfg->rxout.soc_cnt_phyaddr);

	return 0;
}

static int plat_open(struct net_device *pdev, const char *dev_name,
		int id, int flag)
{
	return 0;
}

static void plat_close(struct net_device *pdev, const char *dev_name,
		int flag)
{
	return;
}

static struct aca_ring *plat_to_ring(struct plat_priv *priv, u32 ring_id)
{
	struct aca_ring *ring;

	ring = (struct aca_ring *)&priv->soc_rings;
	ring += ring_id;

	return ring;
}

static int plat_umt_init(u32 id, u32 period, u32 dst_addr)
{
	struct tc_priv *tcpriv;
	struct plat_priv *priv;
	struct aca_ring *ring;
	int i;

	priv = g_plat_priv;
	tcpriv = priv->tc_priv;

	/* first VRX518 device */
	if (!id) {
		ring = (struct aca_ring *)&priv->soc_rings;
		for (i = RXIN_DIR; i <= TXOUT_DIR; i++) {
			ring = plat_to_ring(priv, i);
			ring->umt_dst = (void *)(dst_addr - tcpriv->ep_dev[id].phy_membase
				+ (u32)tcpriv->ep_dev[id].membase + i * 4);
			ring->umt_phydst = dst_addr + i * 4;
			ring->ep_dev_idx = 0;
		}
	} else { /* Second VRX518 device, Only update RX direction */
		ring = plat_to_ring(priv, RXIN_DIR);
		ring->umt_dst
			= (void *)(dst_addr - tcpriv->ep_dev[id].phy_membase
				+ (u32)tcpriv->ep_dev[id].membase
				+ RXIN_DIR * 4);
		ring->ep_dev_idx = id;
		ring = plat_to_ring(priv, RXOUT_DIR);
		ring->umt_dst
			= (void *)(dst_addr - tcpriv->ep_dev[id].phy_membase
				+ (u32)tcpriv->ep_dev[id].membase
				+ RXOUT_DIR * 4);
		ring->ep_dev_idx = id;
	}

	return 0;
}

static void plat_umt_exit(u32 id)
{
	return;
}

static void plat_umt_start(u32 id)
{
	return;
}

static void plat_coc_req(void)
{
	return;
}

static void plat_disable_us(int en)
{
	return;
}

static int plat_get_mib(struct net_device *pdev,
			struct rtnl_link_stats64 *stat)
{
	pr_info("%s is not supported\n", __func__);
	return -ENOTSUPP;
}

static void plat_tc_ops_setup(struct tc_priv *priv)
{
	priv->tc_ops.send = plat_send;
	priv->tc_ops.alloc = plat_mem_alloc;
	priv->tc_ops.free = plat_mem_free;
	priv->tc_ops.dev_reg = plat_open;
	priv->tc_ops.dev_unreg = plat_close;
	priv->tc_ops.umt_init = plat_umt_init;
	priv->tc_ops.umt_exit = plat_umt_exit;
	priv->tc_ops.umt_start = plat_umt_start;
	priv->tc_ops.soc_cfg_get = plat_soc_cfg_get;
	priv->tc_ops.coc_req = plat_coc_req;
	priv->tc_ops.disable_us = plat_disable_us;
	priv->tc_ops.get_mib = plat_get_mib;
}

static int showtime_enter(const unsigned char idx,
		struct port_cell_info *cell_info, void *data)
{
	struct tc_priv *priv;

	if (WARN_ON(idx >= EP_MAX_NUM || cell_info == NULL))
		return -EINVAL;

	priv = g_plat_priv->tc_priv;
	if (priv->showtime[idx] == 1)
		return 0;

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

	if (!priv->showtime[idx])
		return 0;

	priv->showtime[idx] = 0;

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
	//pmac_cfg_set(g_plat_priv->port_id, req_work->tc_mode);
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
	queue_work(system_wq, &g_plat_priv->req_work.work);

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
	tasklet_init(&txout_task, plat_txout_tasklet, 0);
	tasklet_init(&rxout_task, plat_rxout_tasklet, 0);
	irq_init(tc_priv, drv_name);
	//tasklet_init(&priv->coc.coc_task, plat_coc_tasklet, 0);
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
	//tasklet_kill(&g_plat_priv->coc.coc_task);
	plat_dp_exit(g_plat_priv);
	g_plat_priv = NULL;
}
