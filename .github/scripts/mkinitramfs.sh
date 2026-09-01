#!/bin/sh

mkdir -p initramfs/bin
cd initramfs/bin
cp $(command -v busybox.static || command -v busybox) busybox
ln -sf busybox sh

cd ..
cat > init <<EOF
#!/bin/sh
/bin/busybox --install -s /bin

mkdir proc sys
mount -t proc none /proc
mount -t sysfs none /sys

# Print kernel log
dmesg

poweroff -f
EOF

chmod 777 init
find . -print0 | cpio --null -ov -H newc | gzip -9 > ../initramfs.cpio.gz
