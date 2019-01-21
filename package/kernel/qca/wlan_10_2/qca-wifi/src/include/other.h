
struct ctl_table 
{
	const char *procname;		/* Text ID for /proc/sys, or zero */
	void *data;
	int maxlen;
	umode_t mode;
	struct ctl_table *child;	/* Deprecated */
	proc_handler *proc_handler;	/* Callback for text formatting */
	struct ctl_table_poll *poll;
	void *extra1;
	void *extra2;
};
