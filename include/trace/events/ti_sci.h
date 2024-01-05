/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM ti_sci

#if !defined(_TRACE_TI_SCI_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_TI_SCI_H

#include <linux/tracepoint.h>


TRACE_EVENT(ti_sci_xfer_begin,
	TP_PROTO(u16 type, u8 host, u8 seq, u32 flags),
	TP_ARGS(type, host, seq, flags),

	TP_STRUCT__entry(
		__field(u16, type)
		__field(u8, host)
		__field(u8, seq)
		__field(u32, flags)
	),

	TP_fast_assign(
		__entry->type = type;
		__entry->host = host;
		__entry->seq = seq;
		__entry->flags = flags;
	),

	TP_printk("type=%04X host=%02X seq=%02X flags=%08X",
		__entry->type, __entry->host, __entry->seq, __entry->flags)
);

TRACE_EVENT(ti_sci_rx_callback,
	TP_PROTO(u16 type, u8 host, u8 seq, u32 flags),
	TP_ARGS(type, host, seq, flags),

	TP_STRUCT__entry(
		__field(u16, type)
		__field(u8, host)
		__field(u8, seq)
		__field(u32, flags)
	),

	TP_fast_assign(
		__entry->type = type;
		__entry->host = host;
		__entry->seq = seq;
		__entry->flags = flags;
	),

	TP_printk("type=%04X host=%02X seq=%02X flags=%08X",
		__entry->type, __entry->host, __entry->seq, __entry->flags)
);

TRACE_EVENT(ti_sci_xfer_end,
	TP_PROTO(u16 type, u8 host, u8 seq, u32 flags, int status),
	TP_ARGS(type, host, seq, flags, status),

	TP_STRUCT__entry(
		__field(u16, type)
		__field(u8, host)
		__field(u8, seq)
		__field(u32, flags)
		__field(int, status)
	),

	TP_fast_assign(
		__entry->type = type;
		__entry->host = host;
		__entry->seq = seq;
		__entry->flags = flags;
		__entry->status = status;
	),

	TP_printk("type=%04X host=%02X seq=%02X flags=%08X status=%d",
		__entry->type, __entry->host, __entry->seq, __entry->flags, __entry->status)
);


TRACE_EVENT(ti_sci_msg_dump,
	TP_PROTO(u16 type, u8 host, u8 seq, u32 flags, void *buf, size_t len),
	TP_ARGS(type, host, seq, flags, buf, len),

	TP_STRUCT__entry(
		__field(u16, type)
		__field(u8, host)
		__field(u8, seq)
		__field(u32, flags)
		__field(size_t, len)
		__dynamic_array(unsigned char, cmd, len)
	),

	TP_fast_assign(
		__entry->type = type;
		__entry->host = host;
		__entry->seq = seq;
		__entry->flags = flags;
		__entry->len = len;
		memcpy(__get_dynamic_array(cmd), buf, __entry->len);
	),

	TP_printk("type=%04X host=%02X seq=%02X flags=%08X data=%s",
		__entry->type, __entry->host, __entry->seq, __entry->flags,
		__print_hex_str(__get_dynamic_array(cmd), __entry->len))
);
#endif /* _TRACE_TI_SCI_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
