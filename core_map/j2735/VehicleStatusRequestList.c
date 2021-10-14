/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603DA.ASN"
 * 	`asn1c -fcompound-names -pdu=all -no-gen-OER`
 */

#include "VehicleStatusRequestList.h"

asn_per_constraints_t asn_PER_type_VehicleStatusRequestList_constr_1 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 5,  5,  1,  32 }	/* (SIZE(1..32)) */,
	0, 0	/* No PER value map */
};
asn_TYPE_member_t asn_MBR_VehicleStatusRequestList_1[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_VehicleStatusRequest,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_VehicleStatusRequestList_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
asn_SET_OF_specifics_t asn_SPC_VehicleStatusRequestList_specs_1 = {
	sizeof(struct VehicleStatusRequestList),
	offsetof(struct VehicleStatusRequestList, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
asn_TYPE_descriptor_t asn_DEF_VehicleStatusRequestList = {
	"VehicleStatusRequestList",
	"VehicleStatusRequestList",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_VehicleStatusRequestList_tags_1,
	sizeof(asn_DEF_VehicleStatusRequestList_tags_1)
		/sizeof(asn_DEF_VehicleStatusRequestList_tags_1[0]), /* 1 */
	asn_DEF_VehicleStatusRequestList_tags_1,	/* Same as above */
	sizeof(asn_DEF_VehicleStatusRequestList_tags_1)
		/sizeof(asn_DEF_VehicleStatusRequestList_tags_1[0]), /* 1 */
	{ 0, &asn_PER_type_VehicleStatusRequestList_constr_1, SEQUENCE_OF_constraint },
	asn_MBR_VehicleStatusRequestList_1,
	1,	/* Single element */
	&asn_SPC_VehicleStatusRequestList_specs_1	/* Additional specs */
};

