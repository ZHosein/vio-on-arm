#ifndef CLI_H
#define CLI_H
#include <string>
#include <vector>


namespace baby_vSLAM{
    struct Command{ //holds cli key-value pairs
        std::string key;
        std::string val;
    };
    inline Command getCommand(char *arg){ //gets a cli key-value from one cli argument
        std::string argument="", value="";
        for(int i=0;i<2;i++) if(*arg=='-') arg++; //read away - or --
    
        while(*arg!='=' && *arg!='\0') argument+=(*arg)++; //everything before the '='
        if(*arg=='=') arg++; //go past the '='
        while(*arg!='\0') value+=(*arg)++;
    
        Command command = {argument, value};
        return command;
    }
    inline Command *findCommand(std::vector<Command> &commands, const std::string &key){
        int i = commands.size()-1;
        while(i>=0 && commands[i].key!=key) i--;
        if(i<0) return nullptr; //the command with that key wasn't found
        return &commands[i]; //found the command that has the key
    }
    inline std::vector<Command> parseCommands(int argc, char **argv){
        std::vector<Command> commands;
        for(int i=1;i<argc;i++) commands.push_back(getCommand(argv[i]));
        return commands;
    }
}
#endif //CLI_H