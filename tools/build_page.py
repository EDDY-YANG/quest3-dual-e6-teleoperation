"""Build English README, local HTML and shared-axis charts from real summaries."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
RESULTS=ROOT/'analysis/results_v4'
SDK_DOWNLOAD_URL='https://github.com/EDDY-YANG/quest3-dual-e6-teleoperation/releases/latest/download/quest-e6-sdk-windows-x64.zip'
COLORS=['#2563eb','#d97706','#7c3aed']
LABELS=['IK\n(no QP)','IK + QP','RangedIK v7\ndirect']

def val(g,metric,key):
    return g['metrics'].get(metric,{}).get(key)

def fmt(x):
    return 'N/A' if x is None else f'{x:.2f}'

def main():
    data=json.loads((RESULTS/'summary.json').read_text());groups=data['groups']
    assert [g['algorithm'] for g in groups]==['ik','ik_qp','ranged_v7']
    assert all(g['sides']==['left'] for g in groups)
    plt.rcParams.update({'font.family':'DejaVu Sans','svg.fonttype':'none','font.size':10})
    fig,axes=plt.subplots(1,3,figsize=(13.8,4.3));fig.patch.set_facecolor('white')
    panels=[('requested_position_mm','Processed-goal tracking','Position error (mm)','rmse','RMSE','p95','P95'),
            ('tcp_speed_mm_s','Measured TCP motion','Estimated speed (mm/s)','p50','P50','p95','P95'),
            ('command_joint_step_deg','Command continuity','Max-axis step per command (deg)','p95','P95','max','Max')]
    for ax,(metric,title,ylabel,k1,label1,k2,label2) in zip(axes,panels):
        top=max(val(g,metric,k2) or 0 for g in groups)
        for i,g in enumerate(groups):
            a,b=val(g,metric,k1),val(g,metric,k2)
            if a is None or b is None:ax.text(i,.1,'N/A',ha='center');continue
            ax.bar(i,a,width=.48,color=COLORS[i],alpha=.84,zorder=2)
            ax.plot([i,i],[a,b],color=COLORS[i],lw=1.3,zorder=3)
            ax.scatter([i],[b],marker='D',s=35,facecolors='white',edgecolors=COLORS[i],linewidths=1.5,zorder=4)
            ax.text(i+.02,b+top*.045,f'{b:.1f}',ha='center',fontsize=9,color='#233247')
            ax.text(i,a/2,f'{a:.1f}',ha='center',va='center',fontsize=10,color='white',fontweight='bold')
        ax.set_title(title,loc='left',fontsize=12,fontweight='bold',pad=18)
        ax.text(0,1.015,f'Bar: {label1}   Diamond: {label2}',transform=ax.transAxes,fontsize=8.5,color='#596879')
        ax.set_ylabel(ylabel);ax.set_xticks(range(3),LABELS);ax.set_ylim(0,top*1.22);ax.set_xlim(-.6,2.6)
        ax.grid(axis='y',color='#e6eaf0',lw=.7,zorder=0);ax.set_axisbelow(True)
        ax.spines[['top','right']].set_visible(False);ax.spines[['left','bottom']].set_color('#b6c2cf');ax.tick_params(color='#b6c2cf')
    fig.subplots_adjust(left=.065,right=.99,top=.84,bottom=.2,wspace=.37)
    fig.text(.065,.025,'Historical left-arm logs | unequal motions and sample sizes | descriptive, not causal improvement',fontsize=9,color='#586779')
    for ext in ['png','svg']:fig.savefig(ROOT/f'assets/performance.{ext}',dpi=190,facecolor='white')
    plt.close(fig)
    rows=[]
    def row(label,values):rows.append((label,values))
    row('Log dates',[', '.join(g['dates']) for g in groups])
    row('Arm records / records with tracking',[f"{g['headset_arm_record_files']} / {g['record_files_with_tracking_samples']}" for g in groups])
    row('Eligible closed-Grip duration (s)',[fmt(g['grip_duration_s']) for g in groups])
    for title,metric,keys in [
        ('Processed-goal position RMSE / P95 (mm)','requested_position_mm',['rmse','p95']),
        ('Accepted-reference position RMSE / P95 (mm)','reference_position_mm',['rmse','p95']),
        ('Processed-goal orientation P95 (deg)','requested_orientation_deg',['p95']),
        ('Accepted-reference orientation P95 (deg)','reference_orientation_deg',['p95']),
        ('Measured TCP speed P50 / P95 (mm/s)','tcp_speed_mm_s',['p50','p95']),
        ('Joint-command step P95 / max (deg)','command_joint_step_deg',['p95','max'])]:
        row(title,[' / '.join(fmt(val(g,metric,key)) for key in keys) for g in groups])
    row('Non-normal STOP events (all Arm records)',[str(g['non_normal_stop_events']) for g in groups])
    row('Recorded queue drops',[str(g['recorded_queue_drops']) for g in groups])
    labels=['IK (no QP)','IK + QP','RangedIK v7 direct']
    table='<table><thead><tr><th>Metric</th>'+''.join(f'<th class="{g["algorithm"]}">{label}<small>Recorded left-arm data</small></th>' for g,label in zip(groups,labels))+'</tr></thead><tbody>'
    table+=''.join('<tr><td>'+label+'</td>'+''.join('<td>'+v+'</td>' for v in values)+'</tr>' for label,values in rows)+'</tbody></table>'
    findings='In these selected left-arm records, v7 has lower processed-goal position RMSE and higher median TCP speed than IK + QP. QP nevertheless has smaller accepted-reference error and substantially smaller command steps; the v7 maximum step remains visible in the figure. These are observational trade-offs, not proof of overall superiority or of a deployed hybrid DLS advantage.'
    page=(ROOT/'page.template.html').read_text(encoding='utf-8').replace('{{METRICS_TABLE}}',table).replace('{{FINDINGS}}',findings)
    page=page.replace('{{INTERFACES_SECTION}}',(ROOT/'sections/interfaces.html').read_text(encoding='utf-8')).replace('{{REFERENCES_SECTION}}',(ROOT/'sections/references.html').read_text(encoding='utf-8'))
    page = page.replace('{{SDK_DOWNLOAD_URL}}', SDK_DOWNLOAD_URL)
    (ROOT/'index.html').write_text(page,encoding='utf-8')
    md='# Quest 3–Based Bimanual Teleoperation of DOBOT E6 Robots\n\n**Yang Taige · The University of Hong Kong · ME capstone project**\n\nAn inspectable VR-to-robot control stack, from first motion to a calibrated bimanual workflow.\n\n![System setup](assets/system_setup.png)\n\n*Data links, power relationships and independent hardware E-stops. E6 appearance uses the official visual meshes; the illustrated pose is not a live measurement.*\n\n![Data flow and control architecture](assets/data_flow.png)\n\n*Current online path: RangedIK v7 direct. No deployed RangedIK-to-DLS cascade is established by these logs.*\n\n![Three-method performance comparison](assets/performance.png)\n\n'
    md=md.replace('![System setup]', f'[![Download SDK · Windows x64](assets/download-sdk.svg)]({SDK_DOWNLOAD_URL})\n\n[SDK source](https://github.com/EDDY-YANG/quest3-dual-e6-teleoperation/tree/main/sdk) · [Grippers](docs/GRIPPER.md) · [Changelog](CHANGELOG.md) · [Installation](docs/SDK_INSTALL.md) · [Terminal examples 01–06](docs/SDK_EXAMPLES.md) · [View the project website](https://eddy-yang.github.io/quest3-dual-e6-teleoperation/) · [▶ Watch videos](https://eddy-yang.github.io/quest3-dual-e6-teleoperation/#development) · [Repository guide](docs/REPOSITORY.md)\n\nThis repository contains the project showcase, documentation and historical-log analysis. The Release ZIP includes all thirteen editable SDK packages and the Windows native solver. Extract it and double-click **OPEN_IN_VSCODE.cmd** on a Windows x64 PC with Python 3.10 and desktop VS Code.\n\n![System setup]', 1)
    md+='| Metric | 🔵 IK (no QP) | 🟠 IK + QP | 🟣 RangedIK v7 direct |\n|---|---:|---:|---:|\n'
    md+='\n'.join('| '+label+' | '+' | '.join(values)+' |' for label,values in rows)+'\n\n'+findings+'\n\n'
    md+='Processed goals, accepted references and raw hand intent are distinct. Command step size measures output continuity, not physical vibration. [Definitions and exclusions](analysis/methods.md) · [Summary CSV](analysis/results_v4/stage_metrics.csv) · [Per-record CSV](analysis/results_v4/per_record_metrics.csv).\n\n'
    md+='**Development footage** — dates supplied by the author; exact clip-to-log alignment has not been established.\n\n'
    md+='<p align="center">'+''.join(f'<a href="https://eddy-yang.github.io/quest3-dual-e6-teleoperation/#development"><img src="assets/videos/poster_{n:02d}.jpg" width="47%" alt="Video {n}: VR workflow, 17 Nov 2026"></a> ' for n in [4,5])+'</p>\n\n'
    md+='<p align="center">'+''.join(f'<a href="https://eddy-yang.github.io/quest3-dual-e6-teleoperation/#development"><img src="assets/videos/poster_{n:02d}.jpg" height="300" alt="Video {n}: early field experiment"></a> ' for n in [1,2,3])+'</p>\n\n'
    md+='The first recordings established robot motion and exposed stalls and control faults. Later work investigated reference smoothing and tracking limitations. The latest VR interface integrates frame calibration, per-arm start return, reconnection, explicit recovery and recording.\n\n[Operation manual](docs/OPERATIONS.md) · [Code walkthrough](docs/CODE_TOUR.md) · [Media provenance](docs/media_manifest.json) · [Publication boundaries](docs/GITHUB_PREPARATION.md)\n'
    md+='\n**PC & VR interface** — [Explore selected functional panels](https://eddy-yang.github.io/quest3-dual-e6-teleoperation/#interfaces). The v0.2.0 workspace shows both mounted grippers around the central controls in an authentic SDK mirror export. The 17 Nov hardware feedback view is retained separately. Expand the function panels for current source-rendered details; no headset-video screenshots are used.\n\n'
    md+='The headset-first workflow supports a single operator: recheck, calibrate, Grip control, Stop and guarded return-to-start. Recovery clears only eligible collision protection; other alarms and hardware stops still require inspection. [Workflow and recovery](docs/HEADSET_WORKFLOW.md).\n\n'
    md+='**Teleoperation frame calibration** — Record each arm’s base +X direction; fixed WebXR up supplies the vertical constraint, and cross products complete a right-handed frame. Fresh Grip anchors map relative hand motion to TCP targets. [Full procedure and equations](docs/CALIBRATION.md).\n\n'
    md+='**Acknowledgments** — We thank the RangedIK authors and the researchers behind damped least-squares IK (DLS). [IEEE references](docs/REFERENCES.md) · [BibTeX](docs/references.bib).\n'
    (ROOT/'README.md').write_text(md,encoding='utf-8')
    from render_docs import build_documents
    build_documents(ROOT)
    print('Built index.html, README.md, document reading pages and performance PNG/SVG from results_v4.')

if __name__=='__main__':main()
