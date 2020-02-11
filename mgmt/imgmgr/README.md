# Transition tool for Apache Mynewt 1.6 bootloader to MCUBoot
This image-manager is for use with mynewt v1.6. 
The mcuboot patch in dir prepares the new bootloader for accepting
v1 images at boot time by allowing the old Magic in the header. 

The new bootloader "image" is created with:

```
newt create-image -v1 XXXX_boot 222.2.12.1
```

from a mynewt 1.7+ repo with the patch applied. 

# Transitioning from Bootloader v1 to MCUBoot (v2)

├── 3dt_node_0-1-5-1.img        - A. An image with image header v1 that can write BL and accept v2 images
├── 3dt_mcuboot_222-2-12-1.img  - B. The new bootloader, encapsulated as an image
├── 3dt_node_0-1-5-2.img        - C. The new image header v2 image
├── 3dt_tag_0-1-5-1.img         - tag-A. An image with image header v1 that can write BL and accept v2 images
├── 3dt_tag_0-1-5-2.img         - tag-C. The new image header v2 image
└── README.md

## To upgrade a board

1. Upload image A and make it active. 
2. Upload bootloader B, it should be automatically written if successfully uploaded
3. Upload image C, it should be automatically made confirmed. 



