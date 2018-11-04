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