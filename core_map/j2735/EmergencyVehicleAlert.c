/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603DA.ASN"
 * 	`asn1c -fcompound-names -pdu=all -no-gen-OER`
 */

#include "EmergencyVehicleAlert.h"

static int
memb_regional_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size >= 1 && size <= 4)) {
		/* Perform validation of the inner elements */
		return SEQUENCE_OF_constraint(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_per_constraints_t asn_PER_type_regional_constr_12 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 2,  2,  1,  4 }	/* (SIZE(1..4)) */,
	0, 0	/* No PER value map */
};
static asn_per_constraints_t asn_PER_memb_regional_constr_12 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 2,  2,  1,  4 }	/* (SIZE(1..4)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_regional_12[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_RegionalExtension_136P0,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_regional_tags_12[] = {
	(ASN_TAG_CLASS_CONTEXT | (10 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_regional_specs_12 = {
	sizeof(struct EmergencyVehicleAlert__regional),
	offsetof(struct EmergencyVehicleAlert__regional, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_regional_12 = {
	"regional",
	"regional",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_regional_tags_12,
	sizeof(asn_DEF_regional_tags_12)
		/sizeof(asn_DEF_regional_tags_12[0]) - 1, /* 1 */
	asn_DEF_regional_tags_12,	/* Same as above */
	sizeof(asn_DEF_regional_tags_12)
		/sizeof(asn_DEF_regional_tags_12[0]), /* 2 */
	{ 0, &asn_PER_type_regional_constr_12, SEQUENCE_OF_constraint },
	asn_MBR_regional_12,
	1,	/* Single element */
	&asn_SPC_regional_specs_12	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_EmergencyVehicleAlert_1[] = {
	{ ATF_POINTER, 2, offsetof(struct EmergencyVehicleAlert, timeStamp),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MinuteOfTheYear,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"timeStamp"
		},
	{ ATF_POINTER, 1, offsetof(struct EmergencyVehicleAlert, id),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TemporaryID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"id"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct EmergencyVehicleAlert, rsaMsg),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RoadSideAlert,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"rsaMsg"
		},
	{ ATF_POINTER, 8, offsetof(struct EmergencyVehicleAlert, responseType),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ResponseType,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"responseType"
		},
	{ ATF_POINTER, 7, offsetof(struct EmergencyVehicleAlert, details),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_EmergencyDetails,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"details"
		},
	{ ATF_POINTER, 6, offsetof(struct EmergencyVehicleAlert, mass),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleMass,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"mass"
		},
	{ ATF_POINTER, 5, offsetof(struct EmergencyVehicleAlert, basicType),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleType,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"basicType"
		},
	{ ATF_POINTER, 4, offsetof(struct EmergencyVehicleAlert, vehicleType),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleGroupAffected,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"vehicleType"
		},
	{ ATF_POINTER, 3, offsetof(struct EmergencyVehicleAlert, responseEquip),
		(ASN_TAG_CLASS_CONTEXT | (8 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_IncidentResponseEquipment,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"responseEquip"
		},
	{ ATF_POINTER, 2, offsetof(struct EmergencyVehicleAlert, responderType),
		(ASN_TAG_CLASS_CONTEXT | (9 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ResponderGroupAffected,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"responderType"
		},
	{ ATF_POINTER, 1, offsetof(struct EmergencyVehicleAlert, regional),
		(ASN_TAG_CLASS_CONTEXT | (10 << 2)),
		0,
		&asn_DEF_regional_12,
		0,
		{ 0, &asn_PER_memb_regional_constr_12,  memb_regional_constraint_1 },
		0, 0, /* No default value */
		"regional"
		},
};
static const int asn_MAP_EmergencyVehicleAlert_oms_1[] = { 0, 1, 3, 4, 5, 6, 7, 8, 9, 10 };
static const ber_tlv_tag_t asn_DEF_EmergencyVehicleAlert_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_EmergencyVehicleAlert_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* timeStamp */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* id */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* rsaMsg */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* responseType */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* details */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* mass */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* basicType */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 }, /* vehicleType */
    { (ASN_TAG_CLASS_CONTEXT | (8 << 2)), 8, 0, 0 }, /* responseEquip */
    { (ASN_TAG_CLASS_CONTEXT | (9 << 2)), 9, 0, 0 }, /* responderType */
    { (ASN_TAG_CLASS_CONTEXT | (10 << 2)), 10, 0, 0 } /* regional */
};
asn_SEQUENCE_specifics_t asn_SPC_EmergencyVehicleAlert_specs_1 = {
	sizeof(struct EmergencyVehicleAlert),
	offsetof(struct EmergencyVehicleAlert, _asn_ctx),
	asn_MAP_EmergencyVehicleAlert_tag2el_1,
	11,	/* Count of tags in the map */
	asn_MAP_EmergencyVehicleAlert_oms_1,	/* Optional members */
	10, 0,	/* Root/Additions */
	11,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_EmergencyVehicleAlert = {
	"EmergencyVehicleAlert",
	"EmergencyVehicleAlert",
	&asn_OP_SEQUENCE,
	asn_DEF_EmergencyVehicleAlert_tags_1,
	sizeof(asn_DEF_EmergencyVehicleAlert_tags_1)
		/sizeof(asn_DEF_EmergencyVehicleAlert_tags_1[0]), /* 1 */
	asn_DEF_EmergencyVehicleAlert_tags_1,	/* Same as above */
	sizeof(asn_DEF_EmergencyVehicleAlert_tags_1)
		/sizeof(asn_DEF_EmergencyVehicleAlert_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_EmergencyVehicleAlert_1,
	11,	/* Elements count */
	&asn_SPC_EmergencyVehicleAlert_specs_1	/* Additional specs */
};

