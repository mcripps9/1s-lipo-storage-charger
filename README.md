# 1s-lipo-storage-charger
Single cell LiPo storage charger for small lithium polymer cells.  It can
be configured to fully charge a small lipo cell, or to set the cell to
a storage voltage by either discharging or charging the cell to a preset
voltage (around 3.8V).

Discharging is self-powered off the cell, so after a flying session, unused
cells can be plugged in for storage without any external power.  For charging,
and external 5V power supply is needed (either a USB charger, or a 5V BEC for
field charging).

Each channel uses a PIC10F322 as brain, an LTC4054 or compatible charge 
controller chip, and an N-channel MOSFET to switch on/off the discharge load.
