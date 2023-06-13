#ifndef CML_GROUPSMANAGER
#define CML_GROUPSMANAGER

#include <cml/config.h>

namespace CML {

    const int MAXGROUPSIZE = sizeof(unsigned int) * 8;

    class GroupsManager {

    public:
        GroupsManager() {
            mCounter = 0;
        }

        int createGroup(std::string name) {
            int group = mCounter++;
            mGroupsName[group] = name;
            return group;
        }

        int getGroupNumber() const {
            return mCounter;
        }

        std::string getName(int group) const {
            return mGroupsName[group];
        }

    private:
        Atomic<int> mCounter;
        std::string mGroupsName[MAXGROUPSIZE];

    };

}

#endif