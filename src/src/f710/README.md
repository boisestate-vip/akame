
## setup

You will need to do some extra work to get this node running.
First add the file 90-hidraw-permissions.rules to the directory
/etc/udev/rules.d/ :
```
sudo cp 90-hidraw-permissions.rules /etc/udev/rules.d/
```

Then add the user to the plugdev group to grant access non-root
to hidraw devices on the system.

```
sudo usermod -aG plugdev $USER
```

Now things should be able to run...
