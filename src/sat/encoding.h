#ifndef ENCODING_H
#define ENCODING_H

#include "data/htn_instance.h"
#include "sat/sat_interface.h"

class Encoding
{
private:
    HtnInstance &_htn;
    SatInterface _sat;
    
public:
    Encoding(HtnInstance &htn) : _htn(htn), _sat(htn.getParams()) {}
};

#endif // ENCODING_H