#include "trig_utils.h"
#include "ext_time_trig.h"

#define GET_PRIV_DATA struct ext_time_trig_info* data = (struct ext_time_trig_info*)b->private_data

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct ext_time_trig_info
{
	struct ubx_chain chain;
    char chain_id[32];
    double* trigger_times;

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct ext_time_trig_port_cache ports;
};



int ext_time_trig_init(ubx_block_t *b)
{
	struct ext_time_trig_info* data =
	        calloc(1, sizeof(struct ext_time_trig_info));
	if ( data==NULL) {
		ubx_err(b, "ext_time_trig: failed to alloc memory");
		return EOUTOFMEM;
	}
	data->trigger_times = NULL;
	b->private_data = data;

	update_port_cache(b, &data->ports);

	sprintf(data->chain_id, "chain0");
	return ubx_config_add(b, data->chain_id, NULL, "struct ubx_triggee");
}


int ext_time_trig_start(ubx_block_t *b)
{
    GET_PRIV_DATA;
    data->chain.tstats_mode = 0;
    data->chain.tstats_skip_first = 0;
    //data->chain.p_tstats = p_tstats;

    data->chain.triggees_len =
        cfg_getptr_triggee(b, data->chain_id, &(data->chain.triggees));
    int ret = ubx_chain_init(&(data->chain), data->chain_id, 1);

    if( data->trigger_times != NULL ) {
        free(data->trigger_times);
        data->trigger_times = NULL;
    }
    data->trigger_times = (double*)malloc(data->chain.triggees_len * sizeof(double));

    for (int i=0; i<(data->chain.triggees_len); i++) {
        int hertz = (data->chain.triggees[i]).every;
        data->trigger_times[i] = 1.0/hertz;
        //printf("%f\n", data->trigger_times[i] );
    }

    return ret;
}


void ext_time_trig_stop(ubx_block_t *b)
{
    //GET_PRIV_DATA;
}


void ext_time_trig_cleanup(ubx_block_t *b)
{
    GET_PRIV_DATA;
    if( data->trigger_times != NULL ) {
        free(data->trigger_times);
        data->trigger_times = NULL;
    }
    ubx_chain_cleanup( &(data->chain) );
	free(b->private_data);
}


void ext_time_trig_step(ubx_block_t *b)
{
    GET_PRIV_DATA;
    double time;
    read_double(data->ports.time, &time);
    for (int i=0; i<(data->chain.triggees_len); i++) {
        if( time >= data->trigger_times[i]) {
            ubx_cblock_step( (data->chain.triggees[i]).b );
            // add a period, so that the block will gets triggered again
            // at the right time
            data->trigger_times[i] += 1.0/(data->chain.triggees[i]).every;
        }

    }
}

