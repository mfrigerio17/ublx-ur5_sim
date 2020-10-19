#include "signal_generator.h"

#include <math.h>

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct signal_generator_info
{
    unsigned int data_len;
    double time;

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct signal_generator_port_cache ports;
};

/* init */
int signal_generator_init(ubx_block_t *b)
{
	int ret = -1;
	const unsigned int* data_len;
	struct signal_generator_info *inf;

	/* allocate memory for the block local state */
	if ((inf = calloc(1, sizeof(struct signal_generator_info)))==NULL) {
		ubx_err(b, "signal_generator: failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}
	b->private_data=inf;
	inf->time = 0;
	update_port_cache(b, &inf->ports);
	ret=0;

    /* config data_len */
    ret = cfg_getptr_uint(b, "data_len", &data_len);
    if (ret < 0)
        goto out;
    inf->data_len = (ret > 0) ? *data_len : 1;

    ret = ubx_outport_resize(inf->ports.out, inf->data_len);
out:
	return ret;
}

/* start */
int signal_generator_start(ubx_block_t *b)
{
	/* struct signal_generator_info *inf = (struct signal_generator_info*) b->private_data; */
        ubx_info(b, "%s", __func__);
	int ret = 0;
	return ret;
}

/* stop */
void signal_generator_stop(ubx_block_t *b)
{
	/* struct signal_generator_info *inf = (struct signal_generator_info*) b->private_data; */
        ubx_info(b, "%s", __func__);
}

/* cleanup */
void signal_generator_cleanup(ubx_block_t *b)
{
	/* struct signal_generator_info *inf = (struct signal_generator_info*) b->private_data; */
        ubx_info(b, "%s", __func__);
	free(b->private_data);
}

/* step */
void signal_generator_step(ubx_block_t *b)
{
    struct signal_generator_info* data = (struct signal_generator_info*)b->private_data;
	double out = 0.2 * sin(data->time);
    double buff[] = { out, out, out, out, out, out };

	write_double_array(data->ports.out , buff, data->data_len);
	data->time += 0.005;
}

