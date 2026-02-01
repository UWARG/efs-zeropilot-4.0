
#pragma once

class ICAN {

protected:
	ICAN() = default;
	
public:
	virtual ~ICAN() = default;
	
	virtual bool routineTasks() = 0;
};
