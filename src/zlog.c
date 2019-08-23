
typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

#define get_unaligned(ptr) \
({ \
struct __attribute__((packed)) { \
typeof(*(ptr)) __v; \
} *__p = (typeof(__p)) (ptr); \
__p->__v; \
})

#define min(x, y) ({                            \
	typeof(x) _min1 = (x);                  \
	typeof(y) _min2 = (y);                  \
	(void) (&_min1 == &_min2);              \
	_min1 < _min2 ? _min1 : _min2; })

#if 1
extern int g_log_level;
#define DUMP_PREFIX_ADDRESS 1
#define DUMP_PREFIX_OFFSET	2

#define p_err(fmt, args...)	printf("[ERR][%s]:\t" fmt , __func__, ##args)
#define p_war(fmt, args...)	printf("[WAR][%s]:\t" fmt , __func__, ##args)
#define p_not(fmt, args...)	printf("[NOT][%s]:\t" fmt , __func__, ##args)
#define p_inf(fmt, args...)	printf("[INF][%s]:\t" fmt , __func__, ##args)
#define p_dbg(fmt, args...)	printf("[DBG][%s]:\t" fmt , __func__, ##args)

#define p_lin()	printf("[%s][%d]:\n", __func__,__LINE__)

#define l_err(fmt, args...) syslog(LOG_ERR,		"[%s]:\t" fmt ,__func__, ##args)
#define l_war(fmt, args...) syslog(LOG_WARNING,	"[%s]:\t" fmt ,__func__, ##args)
#define l_not(fmt, args...) syslog(LOG_NOTICE,	"[%s]:\t" fmt ,__func__, ##args)
#define l_inf(fmt, args...) syslog(LOG_INFO,	"[%s]:\t" fmt ,__func__, ##args)
#define l_dbg(fmt, args...) syslog(LOG_DEBUF,	"[%s]:\t" fmt ,__func__, ##args)


const char hex_asc[] = "0123456789abcdef";
#define hex_asc_lo(x)	hex_asc[((x) & 0x0f)]
#define hex_asc_hi(x)	hex_asc[((x) & 0xf0) >> 4]
/* is x a power of 2? */
#define is_power_of_2(x)	((x) != 0 && (((x) & ((x) - 1)) == 0))

/**
 * hex_dump_to_buffer - convert a blob of data to "hex ASCII" in memory
 * @buf: data blob to dump
 * @len: number of bytes in the @buf
 * @rowsize: number of bytes to print per line; must be 16 or 32
 * @groupsize: number of bytes to print at a time (1, 2, 4, 8; default = 1)
 * @linebuf: where to put the converted data
 * @linebuflen: total size of @linebuf, including space for terminating NUL
 * @ascii: include ASCII after the hex output
 *
 * hex_dump_to_buffer() works on one "line" of output at a time, i.e.,
 * 16 or 32 bytes of input data converted to hex + ASCII output.
 *
 * Given a buffer of u8 data, hex_dump_to_buffer() converts the input data
 * to a hex + ASCII dump at the supplied memory location.
 * The converted output is always NUL-terminated.
 *
 * E.g.:
 *   hex_dump_to_buffer(frame->data, frame->len, 16, 1,
 *			linebuf, sizeof(linebuf), true);
 *
 * example output buffer:
 * 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f  @ABCDEFGHIJKLMNO
 *
 * Return:
 * The amount of bytes placed in the buffer without terminating NUL. If the
 * output was truncated, then the return value is the number of bytes
 * (excluding the terminating NUL) which would have been written to the final
 * string if enough space had been available.
 */
int hex_dump_to_buffer(const void *buf, size_t len, int rowsize, int groupsize,
		       char *linebuf, size_t linebuflen, bool ascii)
{
	const u8 *ptr = (const u8 *)buf;
	int ngroups;
	u8 ch;
	int j, lx = 0;
	int ascii_column;
	int ret;

	if (rowsize != 16 && rowsize != 32)
		rowsize = 16;

	if (len > rowsize)		/* limit to one line at a time */
		len = rowsize;
	if (!is_power_of_2(groupsize) || groupsize > 8)
		groupsize = 1;
	if ((len % groupsize) != 0)	/* no mixed size output */
		groupsize = 1;

	ngroups = len / groupsize;
	ascii_column = rowsize * 2 + rowsize / groupsize + 1;

	if (!linebuflen)
		goto overflow1;

	if (!len)
		goto nil;

	if (groupsize == 8) {
		const u64 *ptr8 = (const u64 *)buf;

		for (j = 0; j < ngroups; j++) {
			ret = snprintf(linebuf + lx, linebuflen - lx,
				       "%s%16.16lx", j ? " " : "",
				       get_unaligned(ptr8 + j));
			if (ret >= linebuflen - lx)
				goto overflow1;
			lx += ret;
		}
	} else if (groupsize == 4) {
		const u32 *ptr4 = (const u32 *)buf;

		for (j = 0; j < ngroups; j++) {
			ret = snprintf(linebuf + lx, linebuflen - lx,
				       "%s%8.8x", j ? " " : "",
				       get_unaligned(ptr4 + j));
			if (ret >= linebuflen - lx)
				goto overflow1;
			lx += ret;
		}
	} else if (groupsize == 2) {
		const u16 *ptr2 = (const u16 *)buf;

		for (j = 0; j < ngroups; j++) {
			ret = snprintf(linebuf + lx, linebuflen - lx,
				       "%s%4.4x", j ? " " : "",
				       get_unaligned(ptr2 + j));
			if (ret >= linebuflen - lx)
				goto overflow1;
			lx += ret;
		}
	} else {
		for (j = 0; j < len; j++) {
			if (linebuflen < lx + 2)
				goto overflow2;
			ch = ptr[j];
			linebuf[lx++] = hex_asc_hi(ch);
			if (linebuflen < lx + 2)
				goto overflow2;
			linebuf[lx++] = hex_asc_lo(ch);
			if (linebuflen < lx + 2)
				goto overflow2;
			linebuf[lx++] = ' ';
		}
		if (j)
			lx--;
	}
	if (!ascii)
		goto nil;

	while (lx < ascii_column) {
		if (linebuflen < lx + 2)
			goto overflow2;
		linebuf[lx++] = ' ';
	}
	for (j = 0; j < len; j++) {
		if (linebuflen < lx + 2)
			goto overflow2;
		ch = ptr[j];
		linebuf[lx++] = (isascii(ch) && isprint(ch)) ? ch : '.';
	}
nil:
	linebuf[lx] = '\0';
	return lx;
overflow2:
	linebuf[lx++] = '\0';
overflow1:
	return ascii ? ascii_column + len : (groupsize * 2 + 1) * ngroups - 1;
}


/**
 * print_hex_dump - print a text hex dump to syslog for a binary blob of data
 * @level: kernel log level (e.g. KERN_DEBUG)
 * @prefix_str: string to prefix each line with;
 *  caller supplies trailing spaces for alignment if desired
 * @prefix_type: controls whether prefix of an offset, address, or none
 *  is printed (%DUMP_PREFIX_OFFSET, %DUMP_PREFIX_ADDRESS, %DUMP_PREFIX_NONE)
 * @rowsize: number of bytes to print per line; must be 16 or 32
 * @groupsize: number of bytes to print at a time (1, 2, 4, 8; default = 1)
 * @buf: data blob to dump
 * @len: number of bytes in the @buf
 * @ascii: include ASCII after the hex output
 *
 * Given a buffer of u8 data, print_hex_dump() prints a hex + ASCII dump
 * to the kernel log at the specified kernel log level, with an optional
 * leading prefix.
 *
 * print_hex_dump() works on one "line" of output at a time, i.e.,
 * 16 or 32 bytes of input data converted to hex + ASCII output.
 * print_hex_dump() iterates over the entire input @buf, breaking it into
 * "line size" chunks to format and print.
 *
 * E.g.:
 *   print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_ADDRESS,
 *		    16, 1, frame->data, frame->len, true);
 *
 * Example output using %DUMP_PREFIX_OFFSET and 1-byte mode:
 * 0009ab42: 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f  @ABCDEFGHIJKLMNO
 * Example output using %DUMP_PREFIX_ADDRESS and 4-byte mode:
 * ffffffff88089af0: 73727170 77767574 7b7a7978 7f7e7d7c  pqrstuvwxyz{|}~.
 */
void print_hex_dump(const char *prefix_str, int prefix_type,
		    int rowsize, int groupsize,
		    const void *buf, size_t len, bool ascii)
{
	const u8 *ptr = (const u8 *)buf;
	int i, linelen, remaining = len;
	char linebuf[32 * 3 + 2 + 32 + 1];

	if (rowsize != 16 && rowsize != 32)
		rowsize = 16;
	printf(prefix_str);
	for (i = 0; i < len; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;

		hex_dump_to_buffer(ptr + i, linelen, rowsize, groupsize,
				   linebuf, sizeof(linebuf), ascii);

		switch (prefix_type) {
		case DUMP_PREFIX_ADDRESS:
			printf("%p: %s\n", ptr + i, linebuf);
			break;
		case DUMP_PREFIX_OFFSET:
			printf("%.8x: %s\n", i, linebuf);
			break;
		default:
			printf("%s\n", linebuf);
			break;
		}
	}
}

/**
 * print_hex_dump_bytes - shorthand form of print_hex_dump() with default params
 * @prefix_str: string to prefix each line with;
 *  caller supplies trailing spaces for alignment if desired
 * @prefix_type: controls whether prefix of an offset, address, or none
 *  is printed (%DUMP_PREFIX_OFFSET, %DUMP_PREFIX_ADDRESS, %DUMP_PREFIX_NONE)
 * @buf: data blob to dump
 * @len: number of bytes in the @buf
 *
 * Calls print_hex_dump(), with log level of KERN_DEBUG,
 * rowsize of 16, groupsize of 1, and ASCII output included.
 */
void p_hex(const char *prefix_str, const void *buf, size_t len)
{
	print_hex_dump(prefix_str, DUMP_PREFIX_OFFSET, 16, 1,
		       buf, len, true);
}
#else
#define p_err(fmt, args...)	
#define p_war(fmt, args...)	
#define p_not(fmt, args...)	
#define p_inf(fmt, args...)	
#define p_dbg(fmt, args...)	

#define p_lin()	

#define l_err(fmt, args...) 
#define l_war(fmt, args...) 
#define l_not(fmt, args...) 
#define l_inf(fmt, args...) 
#define l_dbg(fmt, args...) 

void p_hex(const char *prefix_str, const void *buf, size_t len)

#endif

void dump_trb(const char *name,union xhci_trb *trb)
{
	p_dbg("dump_trb %s begin\n",name);
	
	switch (TRB_FIELD_TO_TYPE(trb->trans_event.flags))
	{
		case 2://setup
			{
				p_dbg("trb type:setup\n");
				p_dbg("bmRequsetType:0x%x bRequest:0x%x wValue:0x%x windex:0x%x wLength:0x%x\n"
					,(trb->generic.field[0]&0xFF)
					,(trb->generic.field[0]&0xFF00)>>8
					,(trb->generic.field[0]&0xFFFF0000)>>16
					,(trb->generic.field[1]&0xFFFF)
					,(trb->generic.field[1]&0xFFFF0000)>>16
					);
			}
			break;
		case 3://data
			{
				p_dbg("trb type:data\n");
			}
			break;
		case 4://status
			{
				p_dbg("trb type:status\n");
			}
			break;
		case 12://config endpoint command
			{
				p_dbg("trb type:config endpoint command\n");
			}
			break;
		case 15://stop endpoint command//TRB_STOP_RING
			{
				p_dbg("trb type:stop endpoint command\n");
				p_dbg("slotid:%d sp:%d ep_id:%d c:%d\n"
					,TRB_TO_SLOT_ID(trb->generic.field[3])
					,(trb->generic.field[3]&0x800000)>>23
					,(trb->generic.field[3]&0x1F0000)>>16
					,(trb->generic.field[3]&0x01)
					);
			}
			break;
		case 32://transfer event
			{
				p_dbg("trb type:transfer_event\n");
				p_dbg("TRB_ptr:%p TRB_len:%d completion:%d flags:%d "
					"slot_id:%d ep_id:%d trb_type:%d  event_data:%d c:%d\n"
					,(void **)(uintptr_t)le64_to_cpu(trb->trans_event.buffer)
					,trb->trans_event.transfer_len&0xffffff
					,(trb->trans_event.transfer_len>>24) &0xff
					,trb->trans_event.flags
					,TRB_TO_SLOT_ID(trb->trans_event.flags)
					,TRB_TO_EP_ID(trb->trans_event.flags)
					,TRB_FIELD_TO_TYPE(trb->trans_event.flags)
					,(trb->trans_event.flags&0x04)>>2
					,(trb->trans_event.flags&0x01)
					);

			}
			break;
		
		default:
			p_dbg("unsupport trb type:%d\n",TRB_FIELD_TO_TYPE(trb->trans_event.flags));
		
	}
	p_dbg("trb_generic 0x%08x 0x%08x 0x%08x 0x%08x\n"
				,trb->generic.field[0]
				,trb->generic.field[1]
				,trb->generic.field[2]
				,trb->generic.field[3]);
	p_dbg("dump_trb %s end\n",name);
}


