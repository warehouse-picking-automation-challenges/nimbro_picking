// Hardware UART communication with Host
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#ifndef COMM_H
#define COMM_H

#include<stdint.h>

namespace comm
{
void processByte(uint8_t c);
}


#endif
