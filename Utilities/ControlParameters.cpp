#include"ControlParameters.h"

std::string controlParameterValueKindToString(ControlParameterValueKind kind){
    switch (kind){
        case ControlParameterValueKind::FLOAT:
            return {"FLOAT"};
        case ControlParameterValueKind::DOUBLE:
            return {"DOUBLE"};
        case ControlParameterValueKind::S64:
            return {"S64"};
        case ControlParameterValueKind::VEC3_FLOAT:
            return {"VEC3_FLOAT"};
        case ControlParameterValueKind::VEC3_DOUBLE:
            return {"VEC3_DOUBLE"};
        default:
            return {};
    }
}

ControlParameterValueKind getControlParameterValueKindFromString(const std::string& str){
    if(str == "FLOAT") return ControlParameterValueKind::FLOAT;
    if(str == "DOUBLE") return ControlParameterValueKind::DOUBLE;
    if(str == "S64") return ControlParameterValueKind::S64;
    if(str == "VEC3_FLOAT") return ControlParameterValueKind::VEC3_FLOAT;
    if(str == "VEC3_DOUBLE") return ControlParameterValueKind::VEC3_DOUBLE;
    return ControlParameterValueKind::FLOAT;
}

ControlParameter::ControlParameter(){
    _value.init();
    strcpy(_name, "");
    _set = false;
    _kind = ControlParameterValueKind::FLOAT;
}

ControlParameter::ControlParameter(const std::string name, ControlParameterValueKind &kind){
    _value.init();
    if(!this->setName(name)) return;
    _set = true;
    _kind = kind;
}

ControlParameter::ControlParameter(const std::string name, ControlParameterValueKind &kind, ControlParameterValue &value){
    _value.init();
    _kind = kind;
    if(!this->setName(name)) return;
    _value.copy(value);
    _set = true;
}

ControlParameter::ControlParameter(const std::string& name, double value) {
    if(!this->setName(name)) return;
    _set = true;
    _value.d = value;
    _kind = ControlParameterValueKind::DOUBLE;
}

ControlParameter::ControlParameter(const std::string& name, float value){
    if(!this->setName(name)) return;
    _set = true;
    _value.f = value;
    _kind = ControlParameterValueKind::FLOAT;
}

ControlParameter::ControlParameter(const std::string& name, long int value) {
    if(!this->setName(name)) return;
    _set = true;
    _value.i = value;
    _kind = ControlParameterValueKind::S64;
}

ControlParameter::ControlParameter(const std::string& name, float *value){
    if(!this->setName(name)) return;
    for (int i = 0; i < 3; i++){
        _value.vec3f[i] = value[i];
    }
    _set = true;
    _kind = ControlParameterValueKind::VEC3_FLOAT;
}

ControlParameter::ControlParameter(const std::string& name, double *value){
    if(!this->setName(name)) return;
    for (int i = 0; i < 3; i++){
        _value.vec3d[i] = value[i];
    }
    _set = true;
    _kind = ControlParameterValueKind::VEC3_DOUBLE;
}

void ControlParameter::setValueKind(ControlParameterValueKind kind){
    _kind = kind;
}

ControlParameterValueKind ControlParameter::getValueKind(){
    return _kind;
}

void ControlParameter::setValue(float value){
    _kind = ControlParameterValueKind::FLOAT;
    _value.f = value;
    _set = true;
}

void ControlParameter::setValue(double value){
    _kind = ControlParameterValueKind::DOUBLE;
    _value.d = value;
    _set = true;
}

void ControlParameter::setValue(long int value){
    _kind = ControlParameterValueKind::S64;
    _value.i = value;
    _set = true;
}

void ControlParameter::setValue(const Vec3<float> &value){
    _kind = ControlParameterValueKind::VEC3_FLOAT;
    for (int i = 0; i < 3; i++){
        _value.vec3f[i] = value[i];
    }
    _set = true;
}

void ControlParameter::setValue(const Vec3<double> &value){
    _kind = ControlParameterValueKind::VEC3_DOUBLE;
    for (int i = 0; i < 3; i++){
        _value.vec3d[i] = value[i];
    }
    _set = true;
}

void ControlParameter::setValue(ControlParameterValueKind kind, ControlParameterValue value){
    if(_kind != kind){
        printf("[Control Parameter]: The parameter type is different with setting type.");
        throw std::runtime_error("Control parameter type mismatch in set");
        return;
    }
    switch(_kind){
        case ControlParameterValueKind::FLOAT:
            _value.f = value.f;
            break;
        case ControlParameterValueKind::DOUBLE:
            _value.d = value.d;
            break;
        case ControlParameterValueKind::S64:
            _value.i = value.i;
            break;
        case ControlParameterValueKind::VEC3_FLOAT:{
            for (int i = 0; i < 3; i++){
                _value.vec3f[i] = value.vec3f[i];
            }
            break;
        }
        case ControlParameterValueKind::VEC3_DOUBLE:{
            for(int i = 0; i < 3; i++){
                _value.vec3d[i] = value.vec3d[i];
            }
            break;
        }
        default:
            break;
    }
    _set = true;
}

/*！
* Get the value of a control parameter.  Does type checking - you must provide
* the correct type.
* @param kind : the kind of the control parameter
* @return the value of the control parameter
*/

ControlParameterValue ControlParameter::getValue(ControlParameterValueKind kind){
    ControlParameterValue value;
    if (kind != _kind){
        printf("[Control Parameter]: The parameter type is different with setting type.");
        throw std::runtime_error("Control parameter type mismatch in get");
    }
    switch (_kind) {
        case ControlParameterValueKind::FLOAT:
            value.f = _value.f;
            break;
        case ControlParameterValueKind::DOUBLE:
            value.d = _value.d;
            break;
        case ControlParameterValueKind::S64:
            value.i = _value.i;
            break;
        case ControlParameterValueKind::VEC3_FLOAT:{
            value.vec3f[0] = _value.vec3f[0];
            value.vec3f[1] = _value.vec3f[1];
            value.vec3f[2] = _value.vec3f[2];
            break;
        }
        case ControlParameterValueKind::VEC3_DOUBLE:{
            value.vec3d[0] = _value.vec3d[0];
            value.vec3d[1] = _value.vec3d[1];
            value.vec3d[2] = _value.vec3d[2];
            break;
        }
        default:
        throw std::runtime_error("Control parameter invalid kind in get");
    }
    return value;
}


double ControlParameter::getDouble(){
    if (_kind != ControlParameterValueKind::DOUBLE){
        throw std::runtime_error("ControlParameter::getDouble(): type error");
    }
    return _value.d;
}

float ControlParameter::getFloat(){
    if (_kind != ControlParameterValueKind::FLOAT){
        throw std::runtime_error("ControlParameter::getFloat(): type error");
    }
    return _value.f;
}

long int ControlParameter::getS64(){
    if (_kind != ControlParameterValueKind::S64){
        throw std::runtime_error("ControlParameter::getS64(): type error");
    }
    return _value.i;
}

Vec3<double> ControlParameter::getVec3d() {
    if (_kind != ControlParameterValueKind::VEC3_DOUBLE){
        throw std::runtime_error("ControlParameter::getVec3d(): type error");
    }
    Vec3<double> value = Vec3<double>::Zero();
    for (int i = 0; i < 3; i++){
        value[i] = _value.vec3d[i];
    }
    return value;
}

Vec3<float> ControlParameter::getVec3f() {
    if (_kind != ControlParameterValueKind::VEC3_FLOAT){
        throw std::runtime_error("ControlParameter::getVec3f(): type error");
    }
    Vec3<float> value = Vec3<float>::Zero();
    for (int i = 0; i < 3; i++){
        value[i] = _value.vec3f[i];
    }
    return value;
}

double ControlParameter::getFromVec3dByIndex(int index){
    if (index > 3 || index < 0){
        throw std::runtime_error("ControlParameter::getFromVec3dByIndex(): Index out of range");
    }
    if (_kind != ControlParameterValueKind::VEC3_DOUBLE){
        throw std::runtime_error("ControlParameter::getFromVec3dByIndex(): type error");
    }
    return _value.vec3d[index];
}

float ControlParameter::getFromVec3fByIndex(int index){
    if (index > 3 || index < 0){
        throw std::runtime_error("ControlParameter::getFromVec3fByIndex(): Index out of range");
    }
    if (_kind != ControlParameterValueKind::VEC3_FLOAT){
        throw std::runtime_error("ControlParameter::getFromVec3fByIndex(): type error");
    }
    return _value.vec3f[index];
}

bool ControlParameter::setName(std::string name){
    if(name.length() > PARAMETER_MAXIMUM_NAME_LENGTH || name.empty()){
        printf("[ControlParameter]: The parameter name size is invalid when construct it. should be in range[1, 16]");
        return false;
    }else{
        strncpy(_name, name.c_str(), PARAMETER_MAXIMUM_NAME_LENGTH);
        return true;
    }
}

std::string ControlParameter::getName(){
    return _name;
}

template<typename T>
void ControlParameter::setValue(T value) {
    if(std::is_same<T, float>::value && _kind == ControlParameterValueKind::FLOAT){
        _value.f = value;
    }
    if (std::is_same<T, double>::value && _kind == ControlParameterValueKind::DOUBLE){
        _value.d = value;
    }
    if (std::is_same<T, long int>::value && _kind == ControlParameterValueKind::S64){
        _value.i = value;
    }
    if(std::is_same<T, Vec3<float>>::value && _kind == ControlParameterValueKind::VEC3_FLOAT){
        for (int i = 0; i < 3; i++){
            _value.vec3f[i] = value[i];
        }
    }
    if (std::is_same<T, Vec3<double>>::value && _kind == ControlParameterValueKind::VEC3_DOUBLE){
        for (int i = 0; i < 3; i++){
            _value.vec3d[i] = value[i];
        }
    }
    throw std::runtime_error("Error Type When using setValue in ControlParameterValue");
}

void ControlParameterCollection::addParameter(ControlParameter* param, const std::string& name){
    if (mapContains(_map, name)){
        printf("[ERROR] ControlParameterCollection %s: tried to add parameter %s twice!\n", _name.c_str(), name.c_str());
        throw std::runtime_error("Control parameter error [" + _name + "]: parameter " + name + " appears twice!");
    }
    _map[name] = param;
}

ControlParameter& ControlParameterCollection::lookup(const std::string& name){
    if(mapContains(_map, name)){
        return *_map[name];
    }else{
        throw std::runtime_error("Control parameter " + name + " wasn't found in parameter collection " + _name);
    }
}

bool ControlParameterCollection::checkIfAllSet(){
    for (auto& kv : _map){
        if (!kv.second->_set){return false;}
    }
    return true;
}

void ControlParameterCollection::clearAllSet(){
    for (auto& kv : _map){
        kv.second->_set = false;
    }
}

void ControlParameterCollection::deleteAll(){
    for(auto& kv : _map) {
        delete kv.second;
    }
    _map.clear();
}
