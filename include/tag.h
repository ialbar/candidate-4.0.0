/*!	\file tag.h
	\brief Identification of the AMC build
*/

#ifndef TAG_H
#define TAG_H

/* AMC tag information */

#define PACKAGE_NAME	"umc"
#define PACKAGE_VERSION	     4    	//Version
#define PACKAGE_REVIEW	     0		//Review
#define PACKAGE_SUBREVISION  0      //Subrevision
#define PACKAGE_TAG		     0	   //Set to 1 if subversion sources come from a tag
								            //Set to 0 if subversion sources do NOT come from a tag

#define VERSION "V" PACKAGE_VERSION "R" PACKAGE_REVISION "." PACKAGE_SUBREVISION
/*! BUILD is the subversion revision number. It includes up to 3 fields: the revision number,
	a '+' sign if there are mixed revisions (in this case the revision number is the highest
	revision number) and a 'M' if there are local modifications (i.e. "238", "243+M", "235M" or "220+"*/
#define BUILD _SVN_REVISION _SVN_MIXED _SVN_MODS

/*CANOpen manufacturer specific protocol*/
#define MANUFACTURER_VERSION      0     //Version
#define MANUFACTURER_REVIEW	      0     //Review
#define MANUFACTURER_SUBREVISION  0x41  //Subrevision

void publishSoftwareVersion(void);

#endif
