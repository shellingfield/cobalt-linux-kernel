/*
 * Functions depending of the byteorder.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995, 1996, 1997 by Ralf Baechle
 *
 * $Id: byteorder.h,v 1.3 1999/01/14 01:49:07 davem Exp $
 */
#ifndef __ASM_MIPS_BYTEORDER_H
#define __ASM_MIPS_BYTEORDER_H

#define __swap32(x) \
	((unsigned long int)((((unsigned long int)(x) & 0x000000ffU) << 24) | \
			     (((unsigned long int)(x) & 0x0000ff00U) <<  8) | \
			     (((unsigned long int)(x) & 0x00ff0000U) >>  8) | \
			     (((unsigned long int)(x) & 0xff000000U) >> 24)))
#define __swap16(x) \
	((unsigned short int)((((unsigned short int)(x) & 0x00ff) << 8) | \
			      (((unsigned short int)(x) & 0xff00) >> 8)))

#if defined (__MIPSEB__)

#ifndef __BIG_ENDIAN
#define __BIG_ENDIAN 4321
#endif

#ifndef __BIG_ENDIAN_BITFIELD
#define __BIG_ENDIAN_BITFIELD
#endif

#define __constant_ntohl(x) (x)
#define __constant_ntohs(x) (x)
#define __constant_htonl(x) (x)
#define __constant_htons(x) (x)

#elif defined (__MIPSEL__)

#ifndef __LITTLE_ENDIAN
#define __LITTLE_ENDIAN 1234
#endif

#ifndef __LITTLE_ENDIAN_BITFIELD
#define __LITTLE_ENDIAN_BITFIELD
#endif

#define __constant_ntohl(x) __swap32(x)
#define __constant_ntohs(x) __swap16(x)
#define __constant_htonl(x) __swap32(x)
#define __constant_htons(x) __swap16(x)

#endif /* defined(__MIPSEL_) */

extern unsigned int ntohl(unsigned int __x);
extern unsigned short int ntohs(unsigned short int __x);
extern unsigned short int htons(unsigned short int __x);
extern unsigned int htonl(unsigned int __x);

extern __inline__ unsigned int ntohl(unsigned int __x)
{
	return __constant_ntohl(__x);
}

extern __inline__ unsigned short int ntohs(unsigned short int __x)
{
	return __constant_ntohs(__x);
}

extern __inline__ unsigned int htonl(unsigned int __x)
{
	return __constant_htonl(__x);
}

extern __inline__ unsigned short int htons(unsigned short int __x)
{
	return __constant_htons(__x);
}

#endif /* __ASM_MIPS_BYTEORDER_H */
