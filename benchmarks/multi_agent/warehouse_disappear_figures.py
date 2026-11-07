"""Standalone disappear-at-target figures and paired target-policy comparisons."""
from pathlib import Path
import numpy as np

SHORT_LABELS = {
    'rrt': 'RRT-Connect', 'k_routes': 'K: uniform',
    'k_routes_width': 'K: bottleneck',
    'k_routes_edge_cost': 'K: λ=1, τ=1',
    'k_routes_edge_cost_l2_t10': 'K: λ=2, τ=10',
}
PATTERNS = ['distributed', 'opposing_bottleneck', 'intersection',
            'symmetric_swap', 'competing_bottlenecks']
PATTERN_LABELS = ['Distributed', 'Opposing\nbottleneck', 'Intersection',
                  'Symmetric\nswap', 'Competing\nbottlenecks']
COLORS = ['#3278a5', '#6a9b58', '#df8433', '#8b62b5', '#c14b52']


def plots(target,tables,new,parked,pairs,order):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    target=Path(target)
    ticks=[SHORT_LABELS[m] for m in order]

    def save(fig,name):
        fig.savefig(target/f'{name}.png',dpi=160,bbox_inches='tight')
        fig.savefig(target/f'{name}.svg',bbox_inches='tight')
        plt.close(fig)

    def style(ax,percent=False):
        ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
        ax.spines[['top','right']].set_visible(False)
        if percent:ax.set_ylim(0,105)
        else:ax.set_ylim(bottom=0)

    # All results below use only disappear-at-target until the comparison plot.
    overall={r['planner']:r for r in tables['overall']}
    metrics=[('fleet_success_rate','All 32 agents finish (%)',100),
             ('mean_agent_completion_fraction','Individual agents finish (%)',100),
             ('successful_makespan_mean_seconds','Successful makespan (s)',1)]
    fig,axes=plt.subplots(1,3,figsize=(17,5),layout='constrained')
    for ax,(metric,title,scale) in zip(axes,metrics):
        values=[overall[m][metric]*scale for m in order]
        bars=ax.bar(range(len(order)),values,color=COLORS)
        labels=[f'{v:.1f}'+('%' if scale==100 else f'\nn={overall[m]["successes"]}')
                for v,m in zip(values,order)]
        ax.bar_label(bars,labels=labels,padding=4,fontsize=9)
        ax.set(title=title,xticks=range(len(order)),xticklabels=ticks)
        ax.tick_params(axis='x',labelrotation=27);style(ax,scale==100)
        if scale==1:ax.set_ylim(0,max(values)*1.25)
    fig.suptitle('Disappear at target · 250 tasks per method · 32 agents per task\n'
                 'Makespan: mean of successful tasks only; cohorts differ')
    save(fig,'disappear_overview')

    width=.8/len(order)
    for metric,title,scale in metrics[:2]:
        fig,axes=plt.subplots(1,2,figsize=(16,5),layout='constrained',sharey=True)
        for ax,name in zip(axes,('warehouse','warehouse2')):
            lookup={(r['traffic_pattern'],r['planner']):r for r in tables['by_map_pattern']
                    if r['source_map']==name}
            for j,m in enumerate(order):
                ax.bar(np.arange(5)+(j-2)*width,[lookup[p,m][metric]*scale for p in PATTERNS],
                       width*.95,color=COLORS[j],label=SHORT_LABELS[m])
            ax.set(title=name,xticks=range(5),xticklabels=PATTERN_LABELS)
            style(ax,True)
        axes[0].set_ylabel(title);axes[1].legend(fontsize=9)
        fig.suptitle('Disappear at target · 25 tasks per traffic pattern and method on each map')
        save(fig,'disappear_'+('fleet_success' if metric=='fleet_success_rate' else 'agent_completion')+'_by_pattern')

    for metric,name,title in [('fleet_success_rate','disappear_scenario_success','All 32 agents finish (%)'),
            ('mean_agent_completion_fraction','disappear_scenario_completion','Individual agents finish (%)')]:
        fig,axes=plt.subplots(1,2,figsize=(15,14),layout='constrained')
        for ax,map_name in zip(axes,('warehouse','warehouse2')):
            rows=[r for r in tables['by_scenario'] if r['source_map']==map_name]
            scenarios=sorted({r['scenario_id'] for r in rows})
            lookup={(r['scenario_id'],r['planner']):r for r in rows}
            values=np.array([[100*lookup[s,m][metric] for m in order] for s in scenarios])
            image=ax.imshow(values,vmin=0,vmax=100,cmap='YlGnBu',aspect='auto')
            for i in range(len(scenarios)):
                for j in range(len(order)):
                    ax.text(j,i,f'{values[i,j]:.0f}',ha='center',va='center',fontsize=8,
                            color='white' if values[i,j]>55 else '#172533')
            rowlabels=[]
            for s in scenarios:
                r=lookup[s,order[0]]
                rowlabels.append(f'{s} · {r["diameter_meters"]:.1f} m · {r["traffic_pattern"]}')
            ax.set(title=map_name,xticks=range(len(order)),xticklabels=ticks,
                   yticks=range(len(scenarios)),yticklabels=rowlabels)
            ax.tick_params(axis='x',labelrotation=35,labelsize=9)
            ax.tick_params(axis='y',labelsize=8)
        fig.colorbar(image,ax=list(axes),label=title,shrink=.65,pad=.02)
        fig.suptitle('Disappear at target · each cell averages the same 5 start–goal variants')
        save(fig,name)

    fig,axes=plt.subplots(1,2,figsize=(14,5),layout='constrained',sharey=True)
    for ax,map_name in zip(axes,('warehouse','warehouse2')):
        for j,m in enumerate(order):
            group=[r for r in new if r['source_map']==map_name and r['planner']==m]
            times=sorted(r['makespan_seconds'] for r in group if r['success'])
            ax.step([0]+times+[300],[0]+[100*(i+1)/len(group) for i in range(len(times))]
                    +[100*len(times)/len(group)],where='post',color=COLORS[j],label=SHORT_LABELS[m])
        ax.set(title=map_name,xlabel='Simulated time (s)',xlim=(0,300))
        style(ax,True)
    axes[0].set_ylabel('Fraction of all tasks completed (%)');axes[1].legend(fontsize=9)
    fig.suptitle('Disappear at target · fleet completion over time\n'
                 'Denominator includes failed tasks (125 tasks per map and method)')
    save(fig,'disappear_completion_over_time')

    # Explicitly separate target-policy comparisons from standalone results.
    fig,axes=plt.subplots(1,3,figsize=(17,5),layout='constrained')
    old={m:[r for r in parked if r['planner']==m] for m in order}
    for ax,(metric,title,scale) in zip(axes,metrics):
        oldvalues=[]
        for m in order:
            group=old[m]
            values=[r['success'] if metric=='fleet_success_rate' else r['completion_fraction']
                    if metric=='mean_agent_completion_fraction' else r['makespan_seconds']
                    for r in group if metric!='successful_makespan_mean_seconds' or r['success']]
            oldvalues.append(float(np.mean(values))*scale)
        values=[overall[m][metric]*scale for m in order]
        x=np.arange(len(order))
        left=ax.bar(x-.2,oldvalues,.38,color=COLORS,alpha=.45,hatch='///')
        right=ax.bar(x+.2,values,.38,color=COLORS)
        ax.bar_label(left,labels=[f'{v:.1f}' for v in oldvalues],fontsize=8,padding=3)
        ax.bar_label(right,labels=[f'{v:.1f}' for v in values],fontsize=8,padding=3)
        ax.set(title=title,xticks=x,xticklabels=ticks);ax.tick_params(axis='x',labelrotation=27)
        style(ax,scale==100)
        if scale==1:ax.set_ylim(0,max(values+oldvalues)*1.2)
    axes[0].legend(handles=[Patch(facecolor='#888888',alpha=.45,hatch='///',label='Stay at target'),
                            Patch(facecolor='#888888',label='Disappear at target')],fontsize=9)
    fig.suptitle('Same tasks, paths and controller · only target policy changes\n'
                 'Makespan means are conditional on success; successful cohorts differ')
    save(fig,'stay_vs_disappear_overview')

    common={r['planner']:r for r in pairs if r['source_map']=='both'}
    fig,ax=plt.subplots(figsize=(10,5),layout='constrained')
    values=[common[m]['disappear_minus_park_makespan_seconds'] for m in order]
    bars=ax.bar(range(len(order)),values,color=COLORS)
    ax.bar_label(bars,labels=[f'{v:+.2f} s\nn={common[m]["both_successful_tasks"]}'
                              for v,m in zip(values,order)],padding=5,fontsize=10)
    ax.axhline(0,color='#333333',linewidth=.8);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    ax.set(xticks=range(len(order)),xticklabels=ticks,ylabel='Disappear minus stay makespan (s)',
           title='Paired successful tasks only · negative means disappearance was faster')
    ax.margins(y=.25)
    save(fig,'stay_vs_disappear_paired_makespan')
