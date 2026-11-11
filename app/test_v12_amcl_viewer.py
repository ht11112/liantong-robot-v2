"""Run the shipped viewer's rendering code against AMCL and legacy snapshots."""
from pathlib import Path
import shutil
import subprocess
import unittest


class ContinuousAMCLViewerTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'), 'Node is needed to run the actual viewer JavaScript')
    def test_continuous_and_legacy_snapshots(self):
        harness = r"""
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const html=fs.readFileSync(process.argv[1],'utf8'),nodes=new Map(),requests=[];
const operations=[];
function drawing(){
  const values={globalAlpha:1},stack=[];
  return new Proxy(values,{get(target,key){
    if(key in target)return target[key];
    if(key==='save')return()=>stack.push({...target});
    if(key==='restore')return()=>{for(const k of Object.keys(target))delete target[k];Object.assign(target,stack.pop());};
    if(key==='translate')return(x,y)=>{target.x=x;target.y=y;};
    if(key==='rotate')return value=>{target.rotation=value;};
    if(key==='scale')return(x,y)=>{target.scaleX=x;target.scaleY=y;};
    if(key==='stroke')return()=>operations.push({...target});
    return()=>{};
  }});
}
for(const tag of html.split('<script>')[0].match(/<[^>]+>/g)){
  const id=tag.match(/\bid="([^"]+)"/);if(!id)continue;
  const context=drawing(),node={hidden:/\shidden(?:\s|>)/.test(tag),textContent:'',className:'',
    value:'',disabled:/\sdisabled(?:\s|>)/.test(tag),clientWidth:800,clientHeight:500,
    width:0,height:0,setAttribute(){},addEventListener(){},getContext(){return context;}};
  Object.defineProperty(node,'valueAsNumber',{get(){return this.value===''?NaN:Number(this.value);}});
  nodes.set(id[1],node);
}
const $=id=>{assert(nodes.has(id),`Missing HTML element ${id}`);return nodes.get(id);};
for(const id of ['seed-x','seed-y','seed-yaw','goal-x','goal-y'])$(id).value='0';
const sandbox={document:{getElementById:$},window:{devicePixelRatio:1,addEventListener(){}},Image:class{},
  fetch(...args){requests.push(args);throw Error('Rendering must not send a request');},console};
let script=html.match(/<script>([\s\S]*?)<\/script>/)[1];
assert.equal(script.split('render();poll();').length,2);
script=script.replace('render();poll();',`window.viewer={apply(next){state=next;image={};imageKey=next.map?.pgm_sha256||'';render();},screen};`);
vm.runInNewContext(script,sandbox,{filename:'v11_map_viewer.html'});
const map={width:200,height:100,resolution:.05,origin:[1,2,Math.PI/2],pgm_sha256:'map-pgm',map_id:'r5'};
const pose={x:1,y:3,yaw:0},amcl={x:0,y:3,yaw:Math.PI/4};
const base={connected:true,mode:'drive',map,map_image_available:true,csrf_token:'test',can_prepare:true,
  can_start:false,can_stop:true,source_fresh:true,pose,local_pose:{x:0,y:0,yaw:0},task:{phase:'idle'}};
const localization={continuous:true,mode:'amcl',tracking_state:'tracking',amcl_pose:amcl,amcl_age_ms:90,
  sigma_x_m:.02,sigma_y_m:.03,sigma_yaw_rad:Math.PI/180,correction_distance_m:.12,correction_yaw_rad:Math.PI/36};
const apply=state=>{operations.length=0;sandbox.window.viewer.apply(state);};

apply({...base,localization:{mode:'amcl'}});
assert.equal($('localization').textContent,'AMCL 已定位');
assert.equal($('localize').textContent,'单次定位');
assert.equal($('localize').disabled,false);
assert.equal($('amcl-tracking').hidden,true);
assert.equal($('map-arrival-row').hidden,true);
apply({...base,localization:{mode:'manual'},can_localize:false});
assert.equal($('localize').disabled,true);
assert.equal($('zero').disabled,false);

apply({...base,localization,goal:{phase:'executing',map_arrived:null}});
assert.equal($('localization').textContent,'AMCL 持续定位');
assert.equal($('localize').textContent,'AMCL定位并跟踪');
assert.equal($('amcl-tracking').hidden,false);
assert.equal($('vehicle-legend').textContent,'执行位置 / 预览');
assert.equal($('amcl-legend-label').textContent,'AMCL 观测');
assert.equal($('amcl-age').textContent,'90 ms');
assert.equal($('amcl-quality').textContent,'2.0 / 3.0 cm · 1.0°');
assert.equal($('amcl-correction').textContent,'12.0 cm · 5.0°');
assert.match($('pose').textContent,/\(1\.000, 3\.000\)/);
assert.match($('amcl-pose').textContent,/\(0\.000, 3\.000\)/);
let overlay=operations.find(item=>item.strokeStyle==='#b43099');
assert(overlay,'AMCL outline is drawn');
assert(Math.abs(overlay.x-80)<1e-8 && Math.abs(overlay.y-370)<1e-8);
assert(Math.abs(overlay.rotation-Math.PI/4)<1e-8);
assert.equal(overlay.scaleX,1.45);assert.equal(overlay.globalAlpha,1);
assert.equal(operations.at(-1).fillStyle,'#245bd4','The execution marker stays on top');
assert.equal(operations.at(-1).x,80);
assert.equal(operations.at(-1).y,450);

apply({...base,localization:{...localization,tracking_state:'odom_fallback',amcl_age_ms:700},
  task:{phase:'complete',arrival:true,stop_verified:true},
  goal:{phase:'unverified',map_arrived:null,distance_m:.05,reason:'动作完成，等待停车后的 AMCL 确认地图到点'}});
assert.equal($('localization').textContent,'里程计推算');
assert.equal($('amcl-legend-label').textContent,'AMCL 上次观测（已过期）');
assert.equal($('amcl-age').textContent,'700 ms · 已过期');
assert.equal(operations.find(item=>item.strokeStyle==='#b43099').globalAlpha,.28);
assert.equal($('mcu-phase').textContent,'已完成');
assert.equal($('arrival').textContent,'已报告到点');
assert.equal($('task-phase').textContent,'地图到点待验证');
assert.equal($('map-arrival').textContent,'动作完成，地图到点未验证');
assert.equal($('map-distance').textContent,'—');

apply({...base,localization:{...localization,mode:'manual',tracking_state:'manual'}});
assert.equal($('localization').textContent,'手动位姿');
assert.equal($('amcl-legend').hidden,true);
assert.equal(operations.some(item=>item.strokeStyle==='#b43099'),false);
assert.equal($('amcl-pose').textContent,'—');

apply({...base,localization,task:{phase:'failed',error:'MCU 错误 7 / 驱动 72',stop_verified:true},
  goal:{phase:'complete',map_arrived:true,distance_m:.08,reason:'AMCL 已确认地图到点'}});
assert.equal($('task-phase').textContent,'失败','A map result must not hide a real MCU fault');
assert.equal($('task-error').textContent,'MCU 错误 7 / 驱动 72');
assert.equal($('task-error').hidden,false);
assert.equal($('map-arrival').textContent,'已确认（≤10 cm）');
assert.equal($('map-distance').textContent,'8.0 cm');

apply({...base,localization,task:{phase:'failed',error:'MCU 错误 1 / 驱动 0',stop_verified:true},
  goal:{phase:'replanning',map_arrived:false,reason:'已停稳并更新地图参考'}});
assert.equal($('task-phase').textContent,'地图重规划中');
assert.equal($('mcu-phase').textContent,'失败','The raw MCU outcome remains visible');
assert.equal($('task-error').hidden,true);
assert.equal($('goal-status').textContent,'已停稳并更新地图参考');

apply({...base,localization,task:{phase:'failed',error:'MCU 错误 1 / 驱动 0',stop_verified:false},
  goal:{phase:'replanning',map_arrived:false}});
assert.equal($('task-phase').textContent,'失败','An unverified STOP is never treated as a safe pause');
assert.equal($('task-error').hidden,false);

apply({...base,connected:false,localization});
assert.equal($('localization').textContent,'连接断开');
assert.equal($('amcl-legend-label').textContent,'AMCL 上次观测（已过期）');
assert.equal($('stop').disabled,true);
assert.equal(operations.find(item=>item.strokeStyle==='#b43099').globalAlpha,.28);
assert.equal(requests.length,0,'Snapshot rendering must never submit a task or a query');
"""
        result = subprocess.run(
            [shutil.which('node'), '-e', harness, str(Path(__file__).with_name('v11_map_viewer.html'))],
            capture_output=True, text=True, encoding='utf-8', timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
