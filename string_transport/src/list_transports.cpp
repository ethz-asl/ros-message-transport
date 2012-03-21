#include <message_transport/list_transport.h>
#include <std_msgs/String.h>

using namespace message_transport;

int main(int argc, char** argv)
{
	LIST_TRANSPORT("string_transport",std_msgs::String);

	return 0;
}
