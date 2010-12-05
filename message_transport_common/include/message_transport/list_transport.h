#ifndef MESSAGE_TRANSPORT_LIST_TRANSPORT_H
#define MESSAGE_TRANSPORT_LIST_TRANSPORT_H


#include <message_transport/publisher_plugin.h>
#include <message_transport/subscriber_plugin.h>
#include <pluginlib/class_loader.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <map>


namespace message_transport {

	// using namespace pluginlib;

#define LIST_TRANSPORT(PACKAGE,CLASS) {\
	ros::init(argc, argv, "list_transport_"PACKAGE); \
    ListTransport<CLASS> l;l.run(PACKAGE,#CLASS);}

	template <class M>
		class ListTransport
		{
			public:

				enum PluginStatus {SUCCESS, CREATE_FAILURE, LIB_LOAD_FAILURE, DOES_NOT_EXIST};

				/// \cond
				struct TransportDesc
				{
					TransportDesc()
						: pub_status(DOES_NOT_EXIST), sub_status(DOES_NOT_EXIST)
					{}

					std::string package_name;
					std::string pub_name;
					PluginStatus pub_status;
					std::string sub_name;
					PluginStatus sub_status;
				};
				/// \endcond

				ListTransport() {}
				~ListTransport() {}

				void run(const std::string & package_name,const std::string & class_name) {

					pluginlib::ClassLoader<PublisherPlugin< M> > pub_loader(package_name, 
							std::string("message_transport::PublisherPlugin<")+class_name+">");
					pluginlib::ClassLoader<SubscriberPlugin< M> > sub_loader(package_name, 
							std::string("message_transport::SubscriberPlugin<")+class_name+">");
					typedef std::map<std::string, TransportDesc> StatusMap;
					StatusMap transports;

					BOOST_FOREACH(const std::string& lookup_name, pub_loader.getDeclaredClasses()) {
						std::string transport_name = boost::erase_last_copy(lookup_name, "_pub");
						transports[transport_name].pub_name = lookup_name;
						transports[transport_name].package_name = pub_loader.getClassPackage(lookup_name);
						try {
							PublisherPlugin<M> * pub = pub_loader.createClassInstance(lookup_name);
							transports[transport_name].pub_status = SUCCESS;
							delete pub;
						}
						catch (const pluginlib::LibraryLoadException& e) {
							transports[transport_name].pub_status = LIB_LOAD_FAILURE;
						}
						catch (const pluginlib::CreateClassException& e) {
							transports[transport_name].pub_status = CREATE_FAILURE;
						}
					}

					BOOST_FOREACH(const std::string& lookup_name, sub_loader.getDeclaredClasses()) {
						std::string transport_name = boost::erase_last_copy(lookup_name, "_sub");
						transports[transport_name].sub_name = lookup_name;
						transports[transport_name].package_name = sub_loader.getClassPackage(lookup_name);
						try {
							SubscriberPlugin<M> * sub = sub_loader.createClassInstance(lookup_name);
							transports[transport_name].sub_status = SUCCESS;
							delete sub;
						}
						catch (const pluginlib::LibraryLoadException& e) {
							transports[transport_name].sub_status = LIB_LOAD_FAILURE;
						}
						catch (const pluginlib::CreateClassException& e) {
							transports[transport_name].sub_status = CREATE_FAILURE;
						}
					}

					bool problem_package = false;
					printf("Declared transports:\n");
					BOOST_FOREACH(const typename StatusMap::value_type& value, transports) {
						const TransportDesc& td = value.second;
						printf("%s",value.first.c_str());
						if ((td.pub_status == CREATE_FAILURE || td.pub_status == LIB_LOAD_FAILURE) ||
								(td.sub_status == CREATE_FAILURE || td.sub_status == LIB_LOAD_FAILURE)) {
							printf(" (*): Not available. Try 'rosmake %s'.", td.package_name.c_str());
							problem_package = true;
						}
						printf("\n");
					}
#if 0
					if (problem_package)
						printf("(*) \n");
#endif

					printf("\nDetails:\n");
					BOOST_FOREACH(const typename StatusMap::value_type& value, transports) {
						const TransportDesc& td = value.second;
						printf("----------\n");
						printf("\"%s\"\n", value.first.c_str());
						if (td.pub_status == CREATE_FAILURE || td.sub_status == CREATE_FAILURE) {
							printf("*** Plugins are built, but could not be loaded. The package may need to be rebuilt or may not be compatible with this release of message_common. ***\n");
						}
						else if (td.pub_status == LIB_LOAD_FAILURE || td.sub_status == LIB_LOAD_FAILURE) {
							printf("*** Plugins are not built. ***\n");
						}
						printf(" - Provided by package: %s\n", td.package_name.c_str());
						if (td.pub_status == DOES_NOT_EXIST)
							printf(" - No publisher provided\n");
						else
							printf(" - Publisher: %s\n", pub_loader.getClassDescription(td.pub_name).c_str());
						if (td.sub_status == DOES_NOT_EXIST)
							printf(" - No subscriber provided\n");
						else
							printf(" - Subscriber: %s\n", sub_loader.getClassDescription(td.sub_name).c_str());
					}

				}

		};

};

#endif // MESSAGE_TRANSPORT_LIST_TRANSPORT_H
