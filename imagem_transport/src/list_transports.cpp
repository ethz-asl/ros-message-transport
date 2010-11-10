#include <message_transport/list_transport.h>
#include <sensor_msgs/Image.h>

using namespace message_transport;

int main(int argc, char** argv)
{
	LIST_TRANSPORT("imagem_transport",sensor_msgs::Image);

	return 0;
}
