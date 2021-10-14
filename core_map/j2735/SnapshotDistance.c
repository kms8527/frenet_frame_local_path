/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603DA.ASN"
 * 	`asn1c -fcompound-names -pdu=all -no-gen-OER`
 */

#include "SnapshotDistance.h"

asn_TYPE_member_t asn_MBR_SnapshotDistance_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct SnapshotDistance, distance1),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GrossDistance,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"distance1"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SnapshotDistance, speed1),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GrossSpeed,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"speed1"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SnapshotDistance, distance2),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GrossDistance,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"distance2"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SnapshotDistance, speed2),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GrossSpeed,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"speed2"
		},
};
static const ber_tlv_tag_t asn_DEF_SnapshotDistance_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_SnapshotDistance_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* distance1 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* speed1 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* distance2 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* speed2 */
};
asn_SEQUENCE_specifics_t asn_SPC_SnapshotDistance_specs_1 = {
	sizeof(struct SnapshotDistance),
	offsetof(struct SnapshotDistance, _asn_ctx),
	asn_MAP_SnapshotDistance_tag2el_1,
	4,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_SnapshotDistance = {
	"SnapshotDistance",
	"SnapshotDistance",
	&asn_OP_SEQUENCE,
	asn_DEF_SnapshotDistance_tags_1,
	sizeof(asn_DEF_SnapshotDistance_tags_1)
		/sizeof(asn_DEF_SnapshotDistance_tags_1[0]), /* 1 */
	asn_DEF_SnapshotDistance_tags_1,	/* Same as above */
	sizeof(asn_DEF_SnapshotDistance_tags_1)
		/sizeof(asn_DEF_SnapshotDistance_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_SnapshotDistance_1,
	4,	/* Elements count */
	&asn_SPC_SnapshotDistance_specs_1	/* Additional specs */
};

