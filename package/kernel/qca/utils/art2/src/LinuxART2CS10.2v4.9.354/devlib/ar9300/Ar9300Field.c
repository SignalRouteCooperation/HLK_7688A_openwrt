


#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "Field.h"
#include "Ar9300Field.h"


#include "osprey_reg_map_ART_template.h"


void Ar9300FieldSelect()
{
	FieldSelect(_F,sizeof(_F)/sizeof(_F[0]));
}


