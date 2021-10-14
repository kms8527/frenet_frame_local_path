/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603DA.ASN"
 * 	`asn1c -fcompound-names -pdu=all -no-gen-OER`
 */

#ifndef	_ResponseType_H_
#define	_ResponseType_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ResponseType {
	ResponseType_notInUseOrNotEquipped	= 0,
	ResponseType_emergency	= 1,
	ResponseType_nonEmergency	= 2,
	ResponseType_pursuit	= 3,
	ResponseType_stationary	= 4,
	ResponseType_slowMoving	= 5,
	ResponseType_stopAndGoMovement	= 6
	/*
	 * Enumeration is extensible
	 */
} e_ResponseType;

/* ResponseType */
typedef long	 ResponseType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ResponseType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ResponseType;
extern const asn_INTEGER_specifics_t asn_SPC_ResponseType_specs_1;
asn_struct_free_f ResponseType_free;
asn_struct_print_f ResponseType_print;
asn_constr_check_f ResponseType_constraint;
ber_type_decoder_f ResponseType_decode_ber;
der_type_encoder_f ResponseType_encode_der;
xer_type_decoder_f ResponseType_decode_xer;
xer_type_encoder_f ResponseType_encode_xer;
per_type_decoder_f ResponseType_decode_uper;
per_type_encoder_f ResponseType_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _ResponseType_H_ */
#include <asn_internal.h>
