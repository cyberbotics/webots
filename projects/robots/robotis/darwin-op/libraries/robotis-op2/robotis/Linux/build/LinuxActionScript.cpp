/*
 * LinuxActionScript.cpp
 *
 *  Created on: 2011. 1. 18.
 *      Author: zerom
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>

#include "Action.h"
#include "LinuxActionScript.h"
#include "LinuxMotionTimer.h"

using namespace Robot;

pthread_t LinuxActionScript::m_pthread_id;
pid_t LinuxActionScript::mp3_pid = -1;
bool LinuxActionScript::m_stop = 0;
bool LinuxActionScript::m_is_running = 0;

char* LinuxActionScript::SkipLeading(const char* str)
{
    if(str == NULL) return NULL;

    while(*str != '\0' && *str <= ' ')
        str++;

    return (char*)str;
}

int LinuxActionScript::ParseLine(const char* linebuffer, int* pagenumber, char* filepath)
{
    char *sp, *ep;
    char page[16];
    int len = 0;

    sp = SkipLeading(linebuffer);
    ep = strchr(sp, ',');
    len = ep-sp-1;

    if(*sp != '(') return -1;

    strncpy(page,sp+1,len);
    page[len]='\0';

    *pagenumber = (int)atof(page);

    sp = ep;
    ep = strchr(sp, ')');
    len = ep-sp-1;

    strncpy(filepath, sp+1, len);
    filepath[len]='\0';

    return 0;
}

int LinuxActionScript::ScriptStart(const char* filename)
{
    int result;

    m_pthread_id = -1;
    result = pthread_create(&m_pthread_id, NULL, ScriptThreadProc, (void*)filename);
    if(result < 0)
        fprintf(stderr, "Main Routine thread start fail!!\n");
    pthread_detach(m_pthread_id);

    m_is_running = 1;

    return 0;
}

void* LinuxActionScript::ScriptThreadProc(void* data)
{
    FILE* fp;
    int pagenumber;
    char local_buffer[LINE_BUFFERSIZE], filepath[LINE_BUFFERSIZE];

    if((fp = fopen((char*)data,"rt")) == NULL)
        return 0;  /* script file doesn't exist. */

    while(fgets(local_buffer, LINE_BUFFERSIZE, fp) && (m_stop == 0))
    {
        if(ParseLine(local_buffer, &pagenumber, filepath) != -1)
        {
            fprintf(stderr, "Page[%d] : MP3[%s] \n", pagenumber, filepath);
            PlayMP3(filepath);
            Action::GetInstance()->Start(pagenumber);
            while(Action::GetInstance()->IsRunning())
            {
                if(m_stop == 1)
                {
                    Action::GetInstance()->Stop();
                    while(Action::GetInstance()->IsRunning()) usleep(8000);

                    kill(mp3_pid, SIGKILL);

                    m_is_running = 0;
                    m_stop = 0;
                    return 0;
                }
                else usleep(8000);
            }
            sleep(1);
        }
    }

    m_is_running = 0;
    m_stop = 0;
    return 0;
}

int LinuxActionScript::PlayMP3(const char* filename)
{
    if(mp3_pid != -1)
        kill(mp3_pid, SIGKILL);

    mp3_pid = fork();

    switch(mp3_pid)
    {
    case -1:
        fprintf(stderr, "Fork failed!! \n");
        break;
    case 0:
        fprintf(stderr, "Playing MPEG stream from \"%s\" ...\n", filename);
        execl("/usr/bin/madplay", "madplay", filename, "-q", (char*)0);
        fprintf(stderr, "exec failed!! \n");
        break;
    default:
        break;
    }

    return 1;
}

int LinuxActionScript::PlayMP3Wait(const char* filename)
{
    if(mp3_pid != -1)
        kill(mp3_pid, SIGKILL);

    mp3_pid = fork();

    switch(mp3_pid)
    {
    case -1:
        fprintf(stderr, "Fork failed!! \n");
        break;
    case 0:
        fprintf(stderr, "Playing MPEG stream from \"%s\" ...\n", filename);
        execl("/usr/bin/madplay", "madplay", filename, "-q", (char*)0);
        fprintf(stderr, "exec failed!! \n");
        break;
    default:
        int status;
        waitpid(mp3_pid, &status, 0);
        break;
    }

    return 1;
}

