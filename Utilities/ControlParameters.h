#ifndef CONTROLPAREMETERS_H
#define CONTROLPAREMETERS_H

#include <map>
#include <utility>
#include <cstring>
#include <string>
#include "GamepadCommand.h"

#define PARAMETER_MAXIMUM_NAME_LENGTH 16

/*!
 * Does the unordered map contain the given element?
 */
template <typename T1, typename T2>
bool mapContains(const std::map<T1, T2>& set, T1 key){
  return set.find(key) != set.end();
}

/*!
 * Value of a control parameter
 */
union ControlParameterValue {
	float f;
	double d;
	long int i;
	float vec3f[3];
	double vec3d[3];

	void init(){
		memset(this, 0, sizeof(ControlParameterValue));
	}

	void copy(ControlParameterValue &parameter){
		memcpy(this, &parameter, sizeof(ControlParameterValue));
	}
};

/*!
 * Data types supported for control parameters
 * we use enum class, in case that namespace pollution.
 * like FLOAT and DOUBLE
 */
enum class ControlParameterValueKind:unsigned short int{
	FLOAT = 0,
	DOUBLE = 1,
	S64 = 2,
	VEC3_FLOAT = 3,
	VEC3_DOUBLE = 4
};

ControlParameterValueKind getControlParameterValueKindFromString(const std::string& str);
std::string controlParameterValueKindToString(ControlParameterValueKind kind);

/*
 * Used for foreach
 */
const ControlParameterValueKind ControlParameterValueKinds[] = {
	ControlParameterValueKind::FLOAT, 
	ControlParameterValueKind::DOUBLE, 
	ControlParameterValueKind::S64, 
	ControlParameterValueKind::VEC3_FLOAT, 
	ControlParameterValueKind::VEC3_DOUBLE
};

class ControlParameter{
public:
	bool _set;
private:
	char _name[PARAMETER_MAXIMUM_NAME_LENGTH];
	ControlParameterValueKind _kind;
	ControlParameterValue _value;
public:
	/*!
	 * Default constructor
	 */
	ControlParameter();

	/*!
	 * Used to init a parameter but do not set value for this parameter
	 * @param name : name of parameter
	 * @param kind : type of parameter
	 */
	ControlParameter(const std::string name, ControlParameterValueKind &kind);

    /*!
	 * @param name : name of parameter
	 * @param kind : type of parameter
	 * @param value : value of parameter
	 */
	ControlParameter(const std::string name, ControlParameterValueKind &kind, ControlParameterValue &value);

	/*!
     * Construct control parameter for a double
     * @param name : name of parameter
     * @param value : double value
     */
	ControlParameter(const std::string& name, double value);

	/*!
     * Construct control parameter for a float
     * @param name : name of parameter
     * @param value : float value
     */
	ControlParameter(const std::string& name, float value);

	/*!
   	 * Construct control parameter for an s64
     * @param name : name of parameter
     * @param value : reference to value
     */
	ControlParameter(const std::string& name, long int value);

	/*!
	 * Construct control parameter for a list of 3 floats
	 * @param name : name of parameter
	 * @param value : pointer of float value
	 */
	ControlParameter(const std::string& name, float *value);

	/*!
	 * Construct control parameter for a list of 3 floats
	 * @param name : name of parameter
	 * @param value : pointer of double value
	 */
	ControlParameter(const std::string& name, double *value);

	void setValueKind(ControlParameterValueKind kind);
	ControlParameterValueKind getValueKind();
	bool setName(std::string name);
	std::string getName();
	
	void setValue(float value);
	void setValue(double value);
	void setValue(long int value);
	void setValue(const Vec3<float> &value);
	void setValue(const Vec3<double> &value);
  	void setValue(ControlParameterValueKind kind, ControlParameterValue value);

    template<typename T>
    void setValue(T value);
	/*!
	 * Get the value of a control parameter.  Does type checking - you must provide
	 * the correct type.
	 * @param kind : the kind of the control parameter
	 * @return the value of the control parameter
	 */
	ControlParameterValue getValue(ControlParameterValueKind kind);

	double getDouble();
	float getFloat();
	long int getS64();
    Vec3<double> getVec3d();
    Vec3<float> getVec3f();

	double getFromVec3dByIndex(int index);
	float getFromVec3fByIndex(int index);
};

/*!
 * ControlParameterCollections contains a map of all the control parameters which facilitates Read Parameters
 * Mainly used in webots robot program
 */
class ControlParameterCollection{
private:
  	std::string _name;
public:
  	std::map<std::string, ControlParameter*> _map;
	explicit ControlParameterCollection(std::string  name) : _name(std::move(name)){}
   	/*!
   	 * Use this to add a parameter for the first time in the
   	 * Throws exception if you try to add a parameter twice.
   	 */
  	void addParameter(ControlParameter* param, const std::string& name);

	/*!
	 * Lookup a control parameter by its name.
	 * This does not modify the set field of the control parameter!
	 *
	 * Throws exception if parameter isn't found
	 */
	ControlParameter& lookup(const std::string& name);

	//!< are all the control parameters initialized?
	bool checkIfAllSet();

	/*!
	 * Mark all parameters as not set
	 */
	void clearAllSet();

	/*!
	 * Remove all parameters
	 */
	void deleteAll();
};

struct ToolToWebotsMessage{
	size_t nParameters = UINT64_MAX;  // parameters from tool to control robots in webots
    GamepadCommand gameCommand;
	ControlParameter parameters[];	  // The first nParameters in the soft array are from Tool, The last nWebotsData are from webots
};

struct WebotsToToolMessage{
	size_t nParameters = UINT64_MAX;  // parameters from tool to control robots in webots
	ControlParameter parameters[];	  // The first nParameters in the soft array are from Tool, The last nWebotsData are from webots
};

#endif  // PROJECT_CONTROLPAREMETERS_H