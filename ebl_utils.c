#include "ebl_utils.h"

#include "application_properties.h"

#include <stdint.h>


int find_app_properties(FILE *pf, ApplicationProperties_t *pDest)
{

	// this magic string indicates the location of application properties struct within an EBL file
	const uint8_t magic[16] = APPLICATION_PROPERTIES_MAGIC;
	uint8_t buf[16];
	size_t ret;
	int retval = -1;

	while(1)
	{

		// read four bytes at once, assuming the magic constant is aligned at 4-byte boundary
		ret =  fread(buf, 1, 4, pf);

		if(ret != 4)
		{
			printf("could not find app properties! (%d)\n", ret);
			return(-1);
		}

		if(memcmp(magic, buf, 4) == 0)
		{
			// read and compare the remaining bytes
			ret =  fread(buf, 1, 12, pf);

			if(ret != 12)
			{
				printf("could not find app properties! (%d)\n", ret);
				return(-1);
			}

			if(memcmp(&(magic[4]), buf, 12) == 0)
			{
				break;
			}
		}

	}

	// rewind 16 bytes
	fseek(pf, -16, SEEK_CUR);

	ret = fread(pDest, sizeof(ApplicationProperties_t), 1, pf);

	if(ret != 1)
	{
		printf("reading app properties failed\n");
		return(-2);
	}

	printf("SDK version: %d.%d.%d\n",
			pDest->app.capabilities >> 24,
			(pDest->app.capabilities >> 16) & 0xFF,
			(pDest->app.capabilities >> 8) & 0xFF);

	printf("image type: ");
	switch(pDest->app.type)
	{
	case APPLICATION_TYPE_BLUETOOTH:
		printf("STACK\n");
		break;
	case APPLICATION_TYPE_BLUETOOTH_APP:
		printf("APP\n");
		break;
	default:
		printf("unknown? (%d)\n", pDest->app.type);
		break;
	}

	return(0);
}
