# Documentación
Carpeta destinada a almacenar todos los documentos asociados al paquete

## Configurar los permisos udev
```
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
sudo touch /etc/udev/rules.d/99-crazyradio.rules
```

Añadir al archivo /etc/udev/rules.d/99-crazyradio.rules:
```
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
```
Para conectar el Crazyflie 2.0 por usb:
```
sudo touch /etc/udev/rules.d/99-crazyradio.rules
```
```
# Crazyflie 2.0
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
```
Recargar udev-rules:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
