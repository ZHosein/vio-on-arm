#ifndef CLI_H
#define CLI_H
#include <string>
#include <vector>


namespace tiny_arm_slam{
    struct Command{ //holds cli key-value pairs
        std::string key;
        std::string val;
    };
    inline Command getCommand(char *arg){ //gets a cli key-value from one cli argument
        std::string argument="", value="";
        for(int i=0;i<2;i++) if(*arg=='-') arg++; //read away - or --
    
        while(*arg!='=' && *arg!='\0') argument+=*arg++; //everything before the '='
        if(*arg=='=') arg++; //go past the '='
        while(*arg!='\0') value+=*arg++;
    
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
    inline void loadingBar(double fraction){
        int progress = (int)(fraction*20), i;
        std::cout<<"\r[";
        for(i=0;i<20;i++){
            if(i>progress) std::cout<<'|';
            else std::cout<<'=';
        }
        std::cout<<']'<<std::flush;
    }
}
#endif //CLI_H