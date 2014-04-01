#pragma once
#include "Logger.h"

#include <aJSON.h>

class Trigger {
    public:

        int t_since;
        int t_until;
        int on_value;
        int off_value;
        char on_cmp;
        char off_cmp;
        int important;
        int sensor;
        int output;
        int idx;
        int state;

        Trigger();
        void init();
        void load(aJsonObject *msg, Logger * loggers[], int index);
        int tick();
        aJsonObject * json(aJsonObject *cnfdata);
        void set_default_state();

    private:
        Logger * _logger;
};

int trigger_load(Trigger triggers[], Logger * loggers[], aJsonObject * cfile, int trgno);
int triggers_load(Trigger triggers[], Logger * loggers[]);
int triggers_save(Trigger triggers[]);
int trigger_save(Trigger triggers[], int idx);
