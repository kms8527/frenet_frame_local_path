/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603DA.ASN"
 * 	`asn1c -fcompound-names -pdu=all -no-gen-OER`
 */

#ifndef	_TermTime_H_
#define	_TermTime_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TermTime */
typedef long	 TermTime_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_TermTime_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_TermTime;
asn_struct_free_f TermTime_free;
asn_struct_print_f TermTime_print;
asn_constr_check_f TermTime_constraint;
ber_type_decoder_f TermTime_decode_ber;
der_type_encoder_f TermTime_encode_der;
xer_type_decoder_f TermTime_decode_xer;
xer_type_encoder_f TermTime_encode_xer;
per_type_decoder_f TermTime_decode_uper;
per_type_encoder_f TermTime_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _TermTime_H_ */
#include <asn_internal.h>
