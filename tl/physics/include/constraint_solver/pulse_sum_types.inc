#pragma once

// Retail: codsrc\tl\physics\include\constraint_solver\pulse_sum_types.inc (header only).
// SM3 forward-declares the four row types; BO1 adds the contact rows and the per-body node.

class pulse_sum_normal;
class pulse_sum_point;
class pulse_sum_angular;
class pulse_sum_wheel;

// TREYBO: BO1 types (all `class` per mangling: ?set@pulse_sum_contact@@,
// ?setup_constraint@rbint@@SAXPAVrigid_body@@PAVpulse_sum_node@@@Z).
class pulse_sum_contact;
class pulse_sum_contact_point;
class pulse_sum_node;
class pulse_sum_constraint_solver;
