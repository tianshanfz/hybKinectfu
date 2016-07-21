#include"MainController.h"
#include"utils/mesh/meshData.h"


int main()
{
	if(false==MainController::instance()->init("config.ini"))
	{
		cout<<"init main controller failed"<<endl;
		return -1;
	}
	else
	{
		cout<<"init main controller finished"<<endl;
	}
	MainController::instance()->mainLoop();
	cout<<"program finished"<<endl;
	return 0;
}

