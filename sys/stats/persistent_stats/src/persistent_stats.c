#include <assert.h>
#include "os/mynewt.h"
#include "stats/stats.h"
#include "persistent_stats/persistent_stats.h"

/* Currently staticly stored here, perhaps better to pass in ? */
static struct persistent_stats_inst _store_inst = {0};
SLIST_HEAD(, pstats_list) g_pstats_registry =
    SLIST_HEAD_INITIALIZER(g_pstats_registry);

static int
persistent_stats_save_entry(struct stats_hdr *hdr, void *arg, char *name,
                            uint16_t stat_off)
{
    struct persistent_stats_inst* s = (struct persistent_stats_inst*)arg;

    if (s->offset + hdr->s_size > s->max_length) {
        return OS_ENOMEM;
    }
    s->save_fn((uint8_t *)hdr + stat_off, hdr->s_size, s->offset, s->save_fn_arg);
    s->offset += hdr->s_size;

    return (0);
}

int
persistent_stats_save(char *name, uint32_t offset)
{
    struct stats_hdr *hdr;
    int rc;
    uint16_t total_size;

    _store_inst.offset = offset;
    assert(_store_inst.save_fn);
    hdr = stats_group_find(name);
    if (!hdr) {
        rc = OS_EINVAL;
        goto err;
    }

    /* Save the total size of the stats first */
    total_size = hdr->s_size * hdr->s_cnt;
    _store_inst.save_fn((uint8_t *)&total_size, sizeof(total_size),
                        _store_inst.offset, _store_inst.save_fn_arg);
    _store_inst.offset += sizeof(total_size);

    /* Save remaining stats entries */
    rc = stats_walk(hdr, persistent_stats_save_entry, (void*)&_store_inst);
    if (rc != 0) {
        goto err;
    }

    return (0);
err:
    return (rc);
}


static int
persistent_stats_load_entry(struct stats_hdr *hdr, void *arg, char *name,
                            uint16_t stat_off)
{
    struct persistent_stats_inst *s = (struct persistent_stats_inst*)arg;

    if (s->max_length && s->offset + hdr->s_size > s->max_length) {
        return OS_ENOMEM;
    }
    s->load_fn((uint8_t *)hdr + stat_off, hdr->s_size, s->offset, s->load_fn_arg);
    s->offset += hdr->s_size;

    return (0);
}


int
persistent_stats_load(char *name, uint32_t offset)
{
    struct stats_hdr *hdr;
    int rc;
    uint16_t calc_total_size, stored_total_size;

    _store_inst.offset = offset;
    assert(_store_inst.load_fn);
    hdr = stats_group_find(name);
    if (!hdr) {
        rc = OS_EINVAL;
        goto err;
    }

    /* Check that the size matches */
    calc_total_size = hdr->s_size * hdr->s_cnt;
    _store_inst.load_fn((uint8_t *)&stored_total_size, sizeof(stored_total_size),
                        _store_inst.offset, _store_inst.load_fn_arg);

    if (calc_total_size != stored_total_size) {
        rc = OS_ERROR;
        goto err;
    }
    _store_inst.offset += sizeof(stored_total_size);

    /* Load remaining stats entries */
    rc = stats_walk(hdr, persistent_stats_load_entry, (void*)&_store_inst);
    if (rc != 0) {
        goto err;
    }

    return (0);
err:
    return (rc);
}

void
persistent_stats_set_save_cb(persistent_stats_cb_func_t cb, void *arg,
                             uint32_t max_length)
{
    _store_inst.save_fn = cb;
    _store_inst.save_fn_arg = arg;
    _store_inst.max_length = max_length;
}

void
persistent_stats_set_load_cb(persistent_stats_cb_func_t cb, void *arg)
{
    _store_inst.load_fn = cb;
    _store_inst.load_fn_arg = arg;
}

void
persistent_stats_load_all()
{
    struct pstats_list *c;
    uint16_t calc_total_size, stored_total_size;
    uint32_t offset = MYNEWT_VAL(PERSISTENT_STATS_OFFSET);

    SLIST_FOREACH(c, &g_pstats_registry, ps_next) {
        /* Check that the size matches */
        calc_total_size = c->shdr->s_size * c->shdr->s_cnt;
        _store_inst.load_fn((uint8_t *)&stored_total_size, sizeof(stored_total_size),
                            offset, _store_inst.load_fn_arg);
        offset += sizeof(stored_total_size);
        /* Only load this stats data if it's size matches */
        if (calc_total_size == stored_total_size) {
            _store_inst.load_fn((uint8_t *)c->shdr + sizeof(struct stats_hdr), stored_total_size,
                                offset, _store_inst.load_fn_arg);
        }
        offset += stored_total_size;
    }
}

void
persistent_stats_save_all()
{
    struct pstats_list *c;
    uint16_t total_size;
    uint32_t offset = MYNEWT_VAL(PERSISTENT_STATS_OFFSET);

    SLIST_FOREACH(c, &g_pstats_registry, ps_next) {
        /* Store the length of this stats first as uint16_t */
        total_size = c->shdr->s_size * c->shdr->s_cnt;
        _store_inst.save_fn((uint8_t *)&total_size, sizeof(total_size),
                            offset, _store_inst.load_fn_arg);
        offset += sizeof(total_size);
        /* Store the stats data */
        _store_inst.save_fn((uint8_t *)c->shdr + sizeof(struct stats_hdr), total_size,
                            offset, _store_inst.load_fn_arg);
        offset += total_size;
    }
}

int
persistent_stats_register(struct pstats_list *ps)
{
    struct pstats_list *cur, *prev;
    int rc;

    /* Don't allow duplicate entries, return an error if this stat
     * is already registered.
     */
    prev = NULL;
    SLIST_FOREACH(cur, &g_pstats_registry, ps_next) {
        prev = cur;
        if (cur->shdr == ps->shdr) {
            rc = -1;
            goto err;
        }
    }

    if (!prev) {
        SLIST_INSERT_HEAD(&g_pstats_registry, ps, ps_next);
    } else {
        SLIST_INSERT_AFTER(prev, ps, ps_next);
    }

    return (0);
err:
    return (rc);
}
