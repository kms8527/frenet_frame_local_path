/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603DA.ASN"
 * 	`asn1c -fcompound-names -pdu=all -no-gen-OER`
 */

#ifndef	_RequestorDescription_H_
#define	_RequestorDescription_H_


#include <asn_application.h>

/* Including external dependencies */
#include "VehicleID.h"
#include "DescriptiveName.h"
#include "TransitVehicleStatus.h"
#include "TransitVehicleOccupancy.h"
#include "DeltaTime.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RequestorType;
struct RequestorPositionVector;
struct RegionalExtension;

/* RequestorDescription */
typedef struct RequestorDescription {
	VehicleID_t	 id;
	struct RequestorType	*type	/* OPTIONAL */;
	struct RequestorPositionVector	*position	/* OPTIONAL */;
	DescriptiveName_t	*name	/* OPTIONAL */;
	DescriptiveName_t	*routeName	/* OPTIONAL */;
	TransitVehicleStatus_t	*transitStatus	/* OPTIONAL */;
	TransitVehicleOccupancy_t	*transitOccupancy	/* OPTIONAL */;
	DeltaTime_t	*transitSchedule	/* OPTIONAL */;
	struct RequestorDescription__regional {
		A_SEQUENCE_OF(struct RegionalExtension) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RequestorDescription_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RequestorDescription;
extern asn_SEQUENCE_specifics_t asn_SPC_RequestorDescription_specs_1;
extern asn_TYPE_member_t asn_MBR_RequestorDescription_1[9];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RequestorType.h"
#include "RequestorPositionVector.h"
#include "RegionalExtension.h"

#endif	/* _RequestorDescription_H_ */
#include <asn_internal.h>
