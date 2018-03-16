/*
 *	LCD, LED, and Button interface for Cobalt	
 *      Andrew Bose
 *
 *
 */

#define RTC_IO_EXTENT	0x10    /*Only really two ports, but...	*/

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/malloc.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/mc146818rtc.h>
#include <linux/netdevice.h>

#include <asm/io.h>
#include <asm/segment.h>
#include <asm/system.h>
#include <linux/delay.h>

#include "lcd.h"

static int lcd_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
                     unsigned long arg);

static int lcd_present = 1;

int led_state = 0;

#define MAX_INTERFACES	8
static linkcheck_func_t linkcheck_callbacks[MAX_INTERFACES];
static void *linkcheck_cookies[MAX_INTERFACES];

int lcd_register_linkcheck_func(int iface_num, void *func, void *cookie)
{
	if (iface_num < 0 ||
	    iface_num >= MAX_INTERFACES ||
	    linkcheck_callbacks[iface_num] != NULL)
		return -1;
	linkcheck_callbacks[iface_num] = (linkcheck_func_t) func;
	linkcheck_cookies[iface_num] = cookie;
	return 0;
}

static int lcd_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
			unsigned long arg)
{
	struct lcd_display button_display;
	unsigned long address, a;
	int retval, index;

	switch (cmd) {
	case LCD_On:
		udelay(150);
		BusyCheck();
		LCDWriteInst(0x0F);
		break;		

	case LCD_Off:
		udelay(150);
		BusyCheck();
		LCDWriteInst(0x08);
		break;

	case LCD_Reset:
		udelay(150);
		LCDWriteInst(0x3F);
		udelay(150);
		LCDWriteInst(0x3F);
		udelay(150);
		LCDWriteInst(0x3F);
		udelay(150);
		LCDWriteInst(0x3F);
		udelay(150);
		LCDWriteInst(0x01);
		udelay(150);
		LCDWriteInst(0x06);
		break;

	case LCD_Clear:
		udelay(150);
		BusyCheck();
       		LCDWriteInst(0x01);     
		break;

	case LCD_Cursor_Left:
		udelay(150);
		BusyCheck();
		LCDWriteInst(0x10);
		break;

	case LCD_Cursor_Right:
		udelay(150);
		BusyCheck();
		LCDWriteInst(0x14);
		break;	

	case LCD_Cursor_Off:
		udelay(150);
                BusyCheck();
                LCDWriteInst(0x0C);
	        break;

        case LCD_Cursor_On:
                udelay(150);
                BusyCheck();
                LCDWriteInst(0x0F);
                break;

        case LCD_Blink_Off:
                udelay(150);
                BusyCheck();
                LCDWriteInst(0x0E);
                break;

	case LCD_Get_Cursor_Pos:{
                int retval;
                struct lcd_display display;

                retval = verify_area(VERIFY_WRITE, (struct lcd_display*)arg,
                                     sizeof(struct lcd_display));
                if (retval != 0 )
                                return retval;

		udelay(150);
                BusyCheck();
		display.cursor_address = ( LCDReadInst ); 
		display.cursor_address = ( display.cursor_address & 0x07F );
		memcpy_tofs((struct lcd_display*)arg, &display, sizeof(struct lcd_display));

		break;
		}


	case LCD_Set_Cursor_Pos: {
                int retval;
                struct lcd_display display;

                retval = verify_area(VERIFY_READ, (struct lcd_display*)arg, sizeof(struct lcd_display));
                if (retval != 0 )
                                return retval;
                memcpy_fromfs(&display, (struct lcd_display*)arg, sizeof(struct lcd_display));

		a = (display.cursor_address | kLCD_Addr ); 

                udelay(150);
                BusyCheck();    
                LCDWriteInst( a );

                break;
		}
	
	case LCD_Get_Cursor: {
                int retval;
                struct lcd_display display;

                retval = verify_area(VERIFY_WRITE, (struct lcd_display*)arg,
                                     sizeof(struct lcd_display));
                if (retval != 0 )
                                return retval;
                udelay(150);
                BusyCheck();    
		display.character = LCDReadData;	

		memcpy_tofs((struct lcd_display*)arg, &display, sizeof(struct lcd_display));
                udelay(150);
                BusyCheck();
                LCDWriteInst(0x10);

		break;
		}

	case LCD_Set_Cursor:{
                int retval;
                struct lcd_display display;
   
                retval = verify_area(VERIFY_READ, (struct lcd_display*)arg, sizeof(struct lcd_display));
                if (retval != 0 )
                                return retval;
                memcpy_fromfs(&display, (struct lcd_display*)arg, sizeof(struct lcd_display));

                udelay(150);
		BusyCheck();    
                LCDWriteData( display.character );
                udelay(150);
                BusyCheck();
                LCDWriteInst(0x10);

                break;
                }


	case LCD_Disp_Left:
		udelay(150);
		BusyCheck();
		LCDWriteInst(0x18);
		break;

	case LCD_Disp_Right:
		udelay(150);
		BusyCheck();
		LCDWriteInst(0x1C);
		break;

	case LCD_Home:
		udelay(150);
		BusyCheck();
		LCDWriteInst(0x02);
		break;

	case LCD_Write: {
	        int retval;
		struct lcd_display display;
   

                retval = verify_area(VERIFY_READ, (struct lcd_display*)arg, sizeof(struct lcd_display));
                if (retval != 0 )
                                return retval;
                memcpy_fromfs(&display, (struct lcd_display*)arg, sizeof(struct lcd_display));
 
		udelay(150);
                BusyCheck();    
                LCDWriteInst(0x80);
		udelay(150);
		BusyCheck();
	
		for (index = 0; index < (display.size1); index++) {
			udelay(150);
			BusyCheck();	
			LCDWriteData( display.line1[index]);
			BusyCheck();	
		}		

		udelay(150);
		BusyCheck();	
		LCDWriteInst(0xC0);	
		udelay(150);
		BusyCheck();	
	
                for (index = 0; index < (display.size2); index++) {
                        udelay(150);
                        BusyCheck();    
                        LCDWriteData( display.line2[index]);
		}
 
		break;	
	}
	
	case LCD_Read: {	
		int retval;
        	struct lcd_display display;

		retval = verify_area(VERIFY_WRITE, (struct lcd_display*)arg,
		                     sizeof(struct lcd_display));
                if (retval != 0 )
                                return retval;
		BusyCheck();
		for (address = kDD_R00; address <= kDD_R01; address++) {
			a = (address | kLCD_Addr );	

			udelay(150);
			BusyCheck();
			LCDWriteInst( a );
			udelay(150);
			BusyCheck();
			display.line1[address] = LCDReadData;
		}

		display.line1[ 0x27 ] = '\0';
	
		for (address = kDD_R10; address <= kDD_R11; address++) {
			a = (address | kLCD_Addr );     
        
			udelay(150);
	 		BusyCheck();
        		LCDWriteInst( a );
       
        		udelay(150);
	 		BusyCheck();
        		display.line2[address - 0x40 ] = LCDReadData;
		 }

		display.line2[ 0x27 ] = '\0';

		memcpy_tofs((struct lcd_display*)arg, &display,
		            sizeof(struct lcd_display));
		break;
	}

//  set all GPIO leds to led_display.leds 

	case LED_Set: {	
		int retval;
		struct lcd_display led_display;
	

		retval = verify_area(VERIFY_READ, (struct lcd_display*)arg, sizeof(struct lcd_display));
	        if (retval != 0 )
			return retval;
	        memcpy_fromfs(&led_display, (struct lcd_display*)arg,
		              sizeof(struct lcd_display));

		led_state = led_display.leds;
		LEDSet(led_state);

        	break;
	}


//  set only bit led_display.leds

        case LED_Bit_Set: {
                int retval, i;
		int bit=1;
                struct lcd_display led_display;


                retval = verify_area(VERIFY_READ, (struct lcd_display*)arg, sizeof(struct lcd_display));
                if (retval != 0 )
                        return retval;
                memcpy_fromfs(&led_display, (struct lcd_display*)arg,
                              sizeof(struct lcd_display));

		for (i=0;i<(int)led_display.leds;i++)
			{
				bit = 2*bit;	
			}

		led_state = led_state | bit;
                LEDSet(led_state);
                break;
        }

//  clear only bit led_display.leds

        case LED_Bit_Clear: {
                int retval, i;
		int bit=1;
                struct lcd_display led_display;


                retval = verify_area(VERIFY_READ, (struct lcd_display*)arg, sizeof(struct lcd_display));
                if (retval != 0 )
                        return retval;
                memcpy_fromfs(&led_display, (struct lcd_display*)arg,
                              sizeof(struct lcd_display));

                for (i=0;i<(int)led_display.leds;i++)
                        {
                                bit = 2*bit;
                        }

		led_state = led_state &  ~bit;
                LEDSet(led_state);
                break;
        }


	case BUTTON_Read: {

		retval = verify_area(VERIFY_WRITE, (struct lcd_display*)arg, sizeof(struct lcd_display));
                if (retval != 0 )
                         return retval;


		button_display.buttons = GPIRead;
                memcpy_tofs((struct lcd_display*)arg, &button_display, sizeof(struct lcd_display));
		break;
	}

        case LINK_Check: {
                retval = verify_area(VERIFY_WRITE, (struct lcd_display*)arg, sizeof(struct lcd_display));
                if (retval != 0 )
                         return retval;

                button_display.buttons = *((volatile unsigned long *) (0xB0100060) );
                memcpy_tofs((struct lcd_display*)arg, &button_display, sizeof(struct lcd_display));
                break;
        }

	case LINK_Check_2: {
		int iface_num;
                retval = verify_area(VERIFY_WRITE, (struct lcd_display*)arg, sizeof(struct lcd_display));
                if (retval != 0 )
                         return retval;

		/* panel-utils should pass in the desired interface status is wanted for
		 * in "buttons" of the structure.  We will set this to non-zero if the
		 * link is in fact up for the requested interface.  --DaveM
		 */
		memcpy_fromfs(&button_display, (struct lcd_display *)arg, sizeof(button_display));
		iface_num = button_display.buttons;
		if (iface_num >= 0 &&
		    iface_num < MAX_INTERFACES &&
		    linkcheck_callbacks[iface_num] != NULL) {
			button_display.buttons =
				linkcheck_callbacks[iface_num](linkcheck_cookies[iface_num]);
		} else {
			button_display.buttons = 0;
		}

                memcpy_tofs((struct lcd_display*)arg, &button_display, sizeof(struct lcd_display));
                break;
	}

//  Erase the flash

	case FLASH_Erase: {

		int ctr=0;

		    // Chip Erase Sequence
		WRITE_FLASH( kFlash_Addr1, kFlash_Data1 );
		WRITE_FLASH( kFlash_Addr2, kFlash_Data2 );
		WRITE_FLASH( kFlash_Addr1, kFlash_Erase3 );
		WRITE_FLASH( kFlash_Addr1, kFlash_Data1 );
		WRITE_FLASH( kFlash_Addr2, kFlash_Data2 );
		WRITE_FLASH( kFlash_Addr1, kFlash_Erase6 );

		printk( "Erasing Flash.\n");

		while ( (!dqpoll(0x00000000,0xFF)) && (!timeout(0x00000000)) ) {
		    ctr++;
		}

		printk("\n");
		printk("\n");
		printk("\n");

		if (READ_FLASH(0x07FFF0)==0xFF) { printk("Erase Successful\r\n"); }
		else if (timeout) { printk("Erase Timed Out\r\n"); }

	break;
	}

// burn the flash 

	case FLASH_Burn: {

		volatile unsigned long burn_addr;
		unsigned long flags;
		int i;
		unsigned char *rom;
		

                struct lcd_display display;

                retval = verify_area(VERIFY_READ, (struct lcd_display*)arg, sizeof(struct lcd_display));
                if (retval != 0 )
                        return retval;
                memcpy_fromfs(&display, (struct lcd_display*)arg, sizeof(struct lcd_display));

		rom = (unsigned char *) kmalloc((128),GFP_ATOMIC);
                if ( rom == NULL ) {
                       printk ("broken\n");
                       return 1;
                   }

		printk("Churning and Burning -");
		save_flags(flags);
		for (i=0; i<FLASH_SIZE; i=i+128) {

			memcpy_fromfs(rom, display.RomImage + i, 128);

			burn_addr = kFlashBase + i;
			cli();
			for ( index = 0; index < ( 128 ) ; index++ ) 
		  	  {

				WRITE_FLASH( kFlash_Addr1, kFlash_Data1 );
		 	    	WRITE_FLASH( kFlash_Addr2, kFlash_Data2 );
		 	    	WRITE_FLASH( kFlash_Addr1, kFlash_Prog );
		 	    	*((volatile unsigned char *)burn_addr) = (volatile unsigned char) rom[index];

		   	 	 while ( (!dqpoll(burn_addr,(volatile unsigned char) rom[index])) && (!timeout(burn_addr)) ) {
		  	   		}
		  	   	burn_addr++;
		  	  }
			restore_flags(flags);
                	if ( *((volatile unsigned char *)(burn_addr-1)) == (volatile unsigned char) rom[index-1]  ) {
               		 } else if (timeout) {
                	    printk("Program timed out\r\n");
               		 }


		}
		kfree(rom);

	break;
	}

//  read the flash all at once 
	
	case FLASH_Read: {

		unsigned char *user_bytes;
                volatile unsigned long read_addr;
                int i;

		user_bytes = &(((struct lcd_display *)arg)->RomImage[0]);

		retval = verify_area(VERIFY_WRITE, user_bytes, FLASH_SIZE);
                if (retval != 0 )
                         return retval;

		printk("Reading Flash");
		for (i=0; i<FLASH_SIZE; i++) {
			unsigned char tmp_byte;
			read_addr = kFlashBase + i;
			tmp_byte = *((volatile unsigned char *)read_addr);
			put_user (tmp_byte, &user_bytes[i]);
		}


	break;
	}





	default:
		return 0;
	break;

	}

	return 0;

}

static int lcd_open(struct inode *inode, struct file *file)
{
	if (!lcd_present)
		return -ENXIO;
	else
		return 0;
}

/* Only RESET or NEXT counts as button pressed */

static inline int button_pressed(void)
{
	unsigned long buttons = GPIRead;

	if ( (buttons == BUTTON_Next) || (buttons == BUTTON_Next_B) || (buttons == BUTTON_Reset_B) )
		return buttons;
	return 0;
}

/* LED daemon sits on this and we wake him up once a key is pressed. */

static int lcd_waiters = 0;

static int lcd_read(struct inode *inode, struct file *file, char *buf, int count)
{
	int buttons_now;

	if(lcd_waiters > 0)
		return -EINVAL;

	lcd_waiters++;
	while(((buttons_now = (int)button_pressed()) == 0) &&
	      !(current->signal & ~current->blocked)) {
		current->state = TASK_INTERRUPTIBLE;
		current->timeout = jiffies + (2 * HZ);
		schedule();
	}
	lcd_waiters--;

	if(current->signal & ~current->blocked)
		return -ERESTARTSYS;
	return buttons_now;
}

/*
 *	The various file operations we support.
 */

static struct file_operations lcd_fops = {
	NULL,		/* No seek */
	lcd_read,
	NULL,		/* No write */ 
	NULL,		/* No readdir */
	NULL,    	/* No poll */
	lcd_ioctl,
	NULL,		/* No mmap */
	lcd_open,
	NULL		/* No release */	
};

static struct miscdevice lcd_dev=
{
	LCD_MINOR,
	"lcd",
	&lcd_fops
};

int lcd_init(void)
{
unsigned long data;

	printk("%s\n", LCD_DRIVER);
	misc_register(&lcd_dev);

	/* Check region? Naaah! Just snarf it up. */
	request_region(RTC_PORT(0), RTC_IO_EXTENT, "lcd");

	udelay(150);
	data = LCDReadData;
	if ( (data & 0x000000FF) == (0x00) ) {
		lcd_present = 0;
		printk("LCD Not Present\n");
	        }
	else {
		lcd_present = 1;
		WRITE_GAL( kGal_DevBank2PReg, kGal_DevBank2Cfg );
		WRITE_GAL( kGal_DevBank3PReg, kGal_DevBank3Cfg );
		}

	return 0;
}


//
// Function: dqpoll
//
// Description:  Polls the data lines to see if the flash is busy
//
// In: address, byte data
//
// Out: 0 = busy, 1 = write or erase complete
//
//

int dqpoll( volatile unsigned long address, volatile unsigned char data ) {

volatile unsigned char dq7;

dq7 = data & 0x80;

return ( (READ_FLASH(address) & 0x80) == dq7  );

}


//
// Function: timeout
//
// Description: Checks to see if erase or write has timed out
//              By polling dq5
//
// In: address
//
//
// Out: 0 = not timed out, 1 = timed out

int timeout( volatile unsigned long address ) {


return (  (READ_FLASH(address) & 0x20) ==  0x20 );

}



