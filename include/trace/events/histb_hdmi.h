/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM histb_hdmi

#if !defined(_TRACE_HISTB_HDMI_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HISTB_HDMI_H

#include <linux/tracepoint.h>

TRACE_EVENT(histb_hdmi_hpd_irq,
	TP_PROTO(u8 ints0, bool hpd_level, bool hpd_active, bool storm),
	TP_ARGS(ints0, hpd_level, hpd_active, storm),
	TP_STRUCT__entry(
		__field(u8, ints0)
		__field(bool, hpd_level)
		__field(bool, hpd_active)
		__field(bool, storm)
	),
	TP_fast_assign(
		__entry->ints0 = ints0;
		__entry->hpd_level = hpd_level;
		__entry->hpd_active = hpd_active;
		__entry->storm = storm;
	),
	TP_printk("ints0=0x%02x hpd_level=%d hpd_active=%d storm=%d",
		  __entry->ints0, __entry->hpd_level,
		  __entry->hpd_active, __entry->storm)
);

TRACE_EVENT(histb_hdmi_recovery,
	TP_PROTO(const char *reason, int ret, unsigned int hdisplay,
		 unsigned int vdisplay, unsigned int vrefresh),
	TP_ARGS(reason, ret, hdisplay, vdisplay, vrefresh),
	TP_STRUCT__entry(
		__string(reason, reason ? reason : "unknown")
		__field(int, ret)
		__field(unsigned int, hdisplay)
		__field(unsigned int, vdisplay)
		__field(unsigned int, vrefresh)
	),
	TP_fast_assign(
		__assign_str(reason);
		__entry->ret = ret;
		__entry->hdisplay = hdisplay;
		__entry->vdisplay = vdisplay;
		__entry->vrefresh = vrefresh;
	),
	TP_printk("reason=%s ret=%d mode=%ux%u@%u",
		  __get_str(reason), __entry->ret,
		  __entry->hdisplay, __entry->vdisplay,
		  __entry->vrefresh)
);

TRACE_EVENT(histb_hdmi_error,
	TP_PROTO(const char *where, int err),
	TP_ARGS(where, err),
	TP_STRUCT__entry(
		__string(where, where ? where : "unknown")
		__field(int, err)
	),
	TP_fast_assign(
		__assign_str(where);
		__entry->err = err;
	),
	TP_printk("where=%s err=%d", __get_str(where), __entry->err)
);

#endif /* _TRACE_HISTB_HDMI_H */

#include <trace/define_trace.h>
