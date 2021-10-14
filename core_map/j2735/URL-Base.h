/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603DA.ASN"
 * 	`asn1c -fcompound-names -pdu=all -no-gen-OER`
 */

#ifndef	_URL_Base_H_
#define	_URL_Base_H_


#include <asn_application.h>

/* Including external dependencies */
#include <IA5String.h>

#ifdef __cplusplus
extern "C" {
#endif

/* URL-Base */
typedef IA5String_t	 URL_Base_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_URL_Base_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_URL_Base;
asn_struct_free_f URL_Base_free;
asn_struct_print_f URL_Base_print;
asn_constr_check_f URL_Base_constraint;
ber_type_decoder_f URL_Base_decode_ber;
der_type_encoder_f URL_Base_encode_der;
xer_type_decoder_f URL_Base_decode_xer;
xer_type_encoder_f URL_Base_encode_xer;
per_type_decoder_f URL_Base_decode_uper;
per_type_encoder_f URL_Base_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _URL_Base_H_ */
#include <asn_internal.h>
