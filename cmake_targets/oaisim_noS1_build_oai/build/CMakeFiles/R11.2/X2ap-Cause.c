/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "X2AP-IEs"
 * 	found in "/home/matthias/openairinterface5g/openair2/X2AP/MESSAGES/ASN1/R11.2/X2AP-IEs.asn"
 * 	`asn1c -gen-PER`
 */

#include "X2ap-Cause.h"

static asn_per_constraints_t asn_PER_type_X2ap_Cause_constr_1 GCC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  0,  3 }	/* (0..3,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_X2ap_Cause_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct X2ap_Cause, choice.radioNetwork),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_X2ap_CauseRadioNetwork,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"radioNetwork"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct X2ap_Cause, choice.transport),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_X2ap_CauseTransport,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"transport"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct X2ap_Cause, choice.protocol),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_X2ap_CauseProtocol,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"protocol"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct X2ap_Cause, choice.misc),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_X2ap_CauseMisc,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"misc"
		},
};
static asn_TYPE_tag2member_t asn_MAP_X2ap_Cause_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* radioNetwork at 172 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* transport at 173 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* protocol at 174 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* misc at 175 */
};
static asn_CHOICE_specifics_t asn_SPC_X2ap_Cause_specs_1 = {
	sizeof(struct X2ap_Cause),
	offsetof(struct X2ap_Cause, _asn_ctx),
	offsetof(struct X2ap_Cause, present),
	sizeof(((struct X2ap_Cause *)0)->present),
	asn_MAP_X2ap_Cause_tag2el_1,
	4,	/* Count of tags in the map */
	0,
	4	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_X2ap_Cause = {
	"X2ap-Cause",
	"X2ap-Cause",
	CHOICE_free,
	CHOICE_print,
	CHOICE_constraint,
	CHOICE_decode_ber,
	CHOICE_encode_der,
	CHOICE_decode_xer,
	CHOICE_encode_xer,
	CHOICE_decode_uper,
	CHOICE_encode_uper,
	CHOICE_decode_aper,
	CHOICE_encode_aper,
	CHOICE_compare,
	CHOICE_outmost_tag,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	&asn_PER_type_X2ap_Cause_constr_1,
	asn_MBR_X2ap_Cause_1,
	4,	/* Elements count */
	&asn_SPC_X2ap_Cause_specs_1	/* Additional specs */
};
