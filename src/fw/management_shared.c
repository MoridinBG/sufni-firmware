#include "management_shared.h"

#include <string.h>

struct management_core_shared management_core_shared;

void management_shared_reset(void) { memset(&management_core_shared, 0, sizeof(management_core_shared)); }