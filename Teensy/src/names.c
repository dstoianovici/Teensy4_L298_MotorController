/*
* This file is used to remap USB parameters for the tensy Board
* feel free to change this to suit your needs
* remember to change the LEN variable to the length of your changes
*/

#include<usb_names.h>

#define MANUFACTURER_NAME	{'M','e','c','h','D','e','s','i','g','n'}
#define MANUFACTURER_NAME_LEN	10
#define PRODUCT_NAME		{'o','p','e','n','_','m','o','t','o','r'}
#define PRODUCT_NAME_LEN	10

struct usb_string_descriptor_struct usb_string_manufacturer_name = {
	2 + MANUFACTURER_NAME_LEN * 2,
	3,
	MANUFACTURER_NAME};

struct usb_string_descriptor_struct usb_string_product_name = {
	2 + PRODUCT_NAME_LEN * 2,
	3,
	PRODUCT_NAME};