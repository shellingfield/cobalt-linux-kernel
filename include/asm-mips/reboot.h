/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1997 by Ralf Baechle
 *
 * Declare variables for rebooting.
 */
#ifndef __ASM_MIPS_REBOOT_H
#define __ASM_MIPS_REBOOT_H

/*
 * Architecture-specific implementation of sys_reboot commands.
 */
extern void machine_restart(char *cmd);
extern void machine_halt(void);
extern void machine_power_off(void);

void (*_machine_restart)(char *command);
void (*_machine_halt)(void);
void (*_machine_power_off)(void);

#endif /* __ASM_MIPS_REBOOT_H */
