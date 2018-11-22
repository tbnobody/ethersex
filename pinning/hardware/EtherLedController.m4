dnl
dnl EtherLedController.m4
dnl
dnl Pin Configuration for 'EtherLedController'.  Edit it to fit your needs.
dnl

/* port the enc28j60 is attached to */
pin(SPI_CS_NET, SPI_CS_HARDWARE)

ifdef(`conf_STATUSLED_POWER', `dnl
pin(STATUSLED_POWER, PA0, OUTPUT)
')dnl

ifdef(`conf_STATUSLED_BOOTED', `dnl
pin(STATUSLED_BOOTED, PA1, OUTPUT)
')dnl

ifdef(`conf_STATUSLED_HB_ACT', `dnl
pin(STATUSLED_HB_ACT, PA2, OUTPUT)
')dnl

pin(PCA9685_OE, PA7, OUTPUT)

ifelse(value_HD44780_CONNECTION,`HD44780_I2CSUPPORT',`dnl
  ifelse(value_HD44780_I2C_PORTEXP,`HD44780_I2C_PCF8574',`dnl
    ifdef(`conf_HD44780_MULTIEN',
      `dnl
         dnl HD44780_PCF8574x_MULTI_MAPPING(ADR, RS, RW, EN1, EN2, DB4, DB5, DB6, DB7)
         HD44780_PCF8574x_MULTI_MAPPING(0x27, 4, 5, 6, 7, 0, 1, 2, 3)',
      `dnl
         dnl HD44780_PCF8574x_MAPPING(ADR, RS, RW, EN, DB4, DB5, DB6, DB7, BL)
		 HD44780_PCF8574x_MAPPING(0x27, 0, 1, 2, 4, 5, 6, 7, 3)'
    )dnl
  ')dnl
')dnl
