#include <mruby.h>
#include <mruby/string.h>
#include <mruby/irep.h>

#include "hoge.c"

void put(char c)
{
	volatile char* lsr = (volatile char*)0xb2600018; // Line status register.
	volatile char* thr = (volatile char*)0xb2600000; // Transmitter holding register.
 
	while(((*lsr) & 0x20) != 0) ; // Wait until THR is empty.
 
	*thr = c;
}
 
mrb_value myputs(mrb_state *mrb, mrb_value self){
        char *ptr;
        mrb_value val;
        mrb_get_args(mrb, "S", &val);
        for (ptr = RSTRING_PTR(val); *ptr != '\0'; ++ptr) {
                put(*ptr);
        }
        put('\r');
        put('\n');
        return mrb_nil_value();
}
 
int main(void)
{

        mrb_state *mrb;
        mrb = mrb_open();
        mrb_define_method(mrb, mrb->object_class,"myputs", myputs,
            MRB_ARGS_REQ(1));
        mrb_load_irep( mrb, bytecode);
        mrb_close(mrb);

	return 0;
}
