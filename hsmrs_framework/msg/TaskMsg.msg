string PARAM_TYPE_INTEGER = "Integer"
string PARAM_TYPE_STRING = "String"
string PARAM_TYPE_BOOLEAN = "Boolean"
int64 NO_PARENT = -1

uint64 id
int64 parent_id
string type
uint64 priority
#string[] param_names
#string[] param_types
string[] param_values
uint64[] subtask_ids
string[] owners