"""Compose the original models without changing robot kinematics or mesh bytes."""
from pathlib import Path
import copy
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]/'dual_kinova3_teleop/assets'

def compose(variant):
    robot=ET.parse(ROOT/'source/robot/robot.xml').getroot()
    robot.find('compiler').set('autolimits','true')
    for mesh in robot.findall('asset/mesh'):mesh.set('file','source/robot/'+mesh.get('file'))
    base=robot.find(".//body[@name='base']")
    base.set('mocap','true');base.set('pos','0 0 0.9')
    for side in ['right','left']:
        name='robotiq' if variant=='robotiq' else 'psyonic_'+side
        path=next((ROOT/'source'/name).glob('*.xml'))
        hand=ET.parse(path).getroot()
        prefix=side+'_gripper_'
        symbols={e.get(k):prefix+e.get(k) for e in hand.iter() for k in ['name','class'] if e.get(k)}
        for elem in hand.iter():
            for key,value in list(elem.attrib.items()):
                if key=='file':elem.set(key,str(Path('source')/name/value))
                elif key in ('name','class','childclass','mesh','material','joint','joint1','joint2','body','body1','body2','site','site1','site2','tendon','actuator'):
                    elem.set(key,symbols.get(value,value))
        mount=robot.find(f".//body[@name='{side}_hand']")
        for body in hand.find('worldbody'):mount.append(copy.deepcopy(body))
        for tag in ['asset','default','actuator','contact','equality','tendon','sensor']:
            section=hand.find(tag)
            if section is None:continue
            target=robot.find(tag)
            if target is None:target=ET.SubElement(robot,tag)
            target.extend(copy.deepcopy(list(section)))
    # Keep reference sites but hide debug axes and the 20-metre guide cylinder.
    for site in robot.iter('site'):site.set('rgba','0 0 0 0')
    visual=ET.SubElement(robot,'visual')
    ET.SubElement(visual,'headlight',ambient='0.35 0.35 0.35',diffuse='0.55 0.55 0.55',specular='0.1 0.1 0.1')
    ET.SubElement(visual,'global',offwidth='1280',offheight='720')
    asset=robot.find('asset')
    ET.SubElement(asset,'texture',name='sky',type='skybox',builtin='gradient',rgb1='0.25 0.32 0.4',rgb2='0.65 0.7 0.75',width='512',height='3072')
    ET.SubElement(asset,'texture',name='floor_texture',type='2d',builtin='checker',rgb1='0.25 0.28 0.31',rgb2='0.32 0.35 0.38',width='512',height='512')
    ET.SubElement(asset,'material',name='floor_material',texture='floor_texture',texrepeat='4 4',reflectance='0.05')
    world=robot.find('worldbody')
    ET.SubElement(world,'geom',name='floor',type='plane',size='5 5 0.1',material='floor_material',group='2',contype='0',conaffinity='0')
    ET.SubElement(world,'light',name='key',pos='1 -2 4',dir='-0.2 0.4 -1',diffuse='0.5 0.5 0.5')
    ET.SubElement(world,'light',name='fill',pos='-2 2 3',dir='0.3 -0.3 -1',diffuse='0.3 0.3 0.3')
    ET.indent(robot)
    ET.ElementTree(robot).write(ROOT/f'scene_{variant}.xml',encoding='unicode')

if __name__=='__main__':
    for variant in ['robotiq','psyonic']:compose(variant)
