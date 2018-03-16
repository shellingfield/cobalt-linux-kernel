/* $Id: param.h,v 1.1.1.1 1997/10/27 22:43:25 davem Exp $ */
#ifndef _ASMSPARC_PARAM_H
#define _ASMSPARC_PARAM_H

#ifndef HZ
#define HZ 100
#endif

#define EXEC_PAGESIZE	8192    /* Thanks for sun4's we carry baggage... */

#ifndef NGROUPS
#define NGROUPS		32
#endif

#ifndef NOGROUP
#define NOGROUP		(-1)
#endif

#define MAXHOSTNAMELEN	64	/* max length of hostname */

#endif
