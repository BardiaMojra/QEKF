





def plot_df(df:pd.DataFrame,
  rows,
  cols,
  title='_',
  fignum=0,
  show=False,
  save=True,
  output_dir='../out/',
  start=0,
  end=None,
  labels=None,
  range_padding:float=0.5,
  figsize=[5,8]):
  figname = get_fignum_str(fignum)

  if end is None:
    end = df.index.size-1
  if labels is None:
    labels = df.columns
  # set plot colors
  cmap = cm.get_cmap('plasma', 25)
  plot_colors = iter(cmap(np.linspace(0, 1, 25)))
  #plot_colors = iter([plt.cm.tab100(i) for i in range(20)])
  # column labels
  t_cols = ['Tx', 'Ty', 'Tz']
  v_cols = ['vx', 'vy', 'vz']
  w_cols = ['wr', 'wp', 'wy']
  q_cols = ['qx', 'qy', 'qz', 'qw']
  loss_cols = ['L1', 'L2']
  # range padding
  pad = 1+range_padding
  # fig
  fig, axs = plt.subplots(nrows=rows, ncols=cols, figsize=figsize, sharex=True, sharey=False)
  for n, ax in enumerate(axs.flatten()):
    col = labels[n]
    ax.plot(df.loc[start:end,col], marker='.',c=next(plot_colors), ms=1, label=col)
    #ticks = [n % 5 == 0, n > end]
    #ax.tick_params(left=ticks[start], bottom=ticks[end])
    ax.set_title(str(col), size=8)
    if col in t_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin(t_cols)]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin(t_cols)]].max(skipna=False)))
    elif col in v_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin(v_cols)]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin(v_cols)]].max(skipna=False)))
    elif col in w_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin(w_cols)]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin(w_cols)]].max(skipna=False)))
    elif col in q_cols:
      ax.set_ylim(-1.1,1.1)
    elif col in loss_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin([col])]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin([col])]].max(skipna=False)))
    else:
      eprint(lhead+'Err--->> column label missing for figure: '+figname+' .....\n\n')
  fig.subplots_adjust(wspace=0.05)
  ax.set_xlabel('time')
  fig.legend()
  # set title
  if title != '_':
    fig.suptitle('{}'.format(title), y = 0.95)
  plt.suptitle(title)
  title = title.replace(' ', '_')
  # save and show image
  if save==True and output_dir is not None:
    fig_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(fig_name, bbox_inches='tight',dpi=400)
    csv_name = output_dir+title
    df.to_csv(csv_name+'.csv', columns=df.columns)
    prt_file_save(lhead+'saving figure: '+fig_name+'.png')
  if show==True: fig.show()
  else: plt.close()
  return
