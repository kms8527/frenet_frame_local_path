/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603DA.ASN"
 * 	`asn1c -fcompound-names -pdu=all -no-gen-OER`
 */

#include "Location-tech.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static asn_per_constraints_t asn_PER_type_Location_tech_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  4,  4,  0,  9 }	/* (0..9,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_Location_tech_value2enum_1[] = {
	{ 0,	16,	"loc-tech-unknown" },
	{ 1,	13,	"loc-tech-GNSS" },
	{ 2,	13,	"loc-tech-DGPS" },
	{ 3,	12,	"loc-tech-RTK" },
	{ 4,	12,	"loc-tech-PPP" },
	{ 5,	14,	"loc-tech-drGPS" },
	{ 6,	15,	"loc-tech-drDGPS" },
	{ 7,	11,	"loc-tech-dr" },
	{ 8,	12,	"loc-tech-nav" },
	{ 9,	14,	"loc-tech-fault" }
	/* This list is extensible */
};
static const unsigned int asn_MAP_Location_tech_enum2value_1[] = {
	2,	/* loc-tech-DGPS(2) */
	1,	/* loc-tech-GNSS(1) */
	4,	/* loc-tech-PPP(4) */
	3,	/* loc-tech-RTK(3) */
	7,	/* loc-tech-dr(7) */
	6,	/* loc-tech-drDGPS(6) */
	5,	/* loc-tech-drGPS(5) */
	9,	/* loc-tech-fault(9) */
	8,	/* loc-tech-nav(8) */
	0	/* loc-tech-unknown(0) */
	/* This list is extensible */
};
static const asn_INTEGER_specifics_t asn_SPC_Location_tech_specs_1 = {
	asn_MAP_Location_tech_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_Location_tech_enum2value_1,	/* N => "tag"; sorted by N */
	10,	/* Number of elements in the maps */
	11,	/* Extensions before this member */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_Location_tech_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_Location_tech = {
	"Location-tech",
	"Location-tech",
	&asn_OP_NativeEnumerated,
	asn_DEF_Location_tech_tags_1,
	sizeof(asn_DEF_Location_tech_tags_1)
		/sizeof(asn_DEF_Location_tech_tags_1[0]), /* 1 */
	asn_DEF_Location_tech_tags_1,	/* Same as above */
	sizeof(asn_DEF_Location_tech_tags_1)
		/sizeof(asn_DEF_Location_tech_tags_1[0]), /* 1 */
	{ 0, &asn_PER_type_Location_tech_constr_1, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_Location_tech_specs_1	/* Additional specs */
};
