/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603DA.ASN"
 * 	`asn1c -fcompound-names -pdu=all -no-gen-OER`
 */

#include "DistanceUnits.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
asn_per_constraints_t asn_PER_type_DistanceUnits_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_DistanceUnits_value2enum_1[] = {
	{ 0,	10,	"centimeter" },
	{ 1,	5,	"cm2-5" },
	{ 2,	9,	"decimeter" },
	{ 3,	5,	"meter" },
	{ 4,	9,	"kilometer" },
	{ 5,	4,	"foot" },
	{ 6,	4,	"yard" },
	{ 7,	4,	"mile" }
};
static const unsigned int asn_MAP_DistanceUnits_enum2value_1[] = {
	0,	/* centimeter(0) */
	1,	/* cm2-5(1) */
	2,	/* decimeter(2) */
	5,	/* foot(5) */
	4,	/* kilometer(4) */
	3,	/* meter(3) */
	7,	/* mile(7) */
	6	/* yard(6) */
};
const asn_INTEGER_specifics_t asn_SPC_DistanceUnits_specs_1 = {
	asn_MAP_DistanceUnits_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_DistanceUnits_enum2value_1,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_DistanceUnits_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_DistanceUnits = {
	"DistanceUnits",
	"DistanceUnits",
	&asn_OP_NativeEnumerated,
	asn_DEF_DistanceUnits_tags_1,
	sizeof(asn_DEF_DistanceUnits_tags_1)
		/sizeof(asn_DEF_DistanceUnits_tags_1[0]), /* 1 */
	asn_DEF_DistanceUnits_tags_1,	/* Same as above */
	sizeof(asn_DEF_DistanceUnits_tags_1)
		/sizeof(asn_DEF_DistanceUnits_tags_1[0]), /* 1 */
	{ 0, &asn_PER_type_DistanceUnits_constr_1, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_DistanceUnits_specs_1	/* Additional specs */
};

