from pathlib import Path

TEMPLATE = """
<mujoco model="legged_robot">
  <option timestep="0.002" gravity="0 0 -9.81"/>  
  <worldbody>
    <body name="base" pos="0 0 {body_height}">
      <geom type="box" size="0.05 0.05 0.02" rgba="0.3 0.3 0.3 1"/>

      <body name="leg" pos="0 0 0">
        <joint name="hip" type="hinge" axis="0 1 0" range="-30 30"/>
        <geom type="capsule" fromto="0 0 0 0 0 -{leg_length}" size="0.01" rgba="0.2 0.6 0.2 1"/>
      </body>

    </body>
  </worldbody>
</mujoco>
"""
#gravity is defined as -ve Z

def generate_xml(leg_length, body_height=0.15, out_path="generated.xml"):
    xml = TEMPLATE.format(
        leg_length=leg_length,
        body_height=body_height
    )

    out_path = Path(out_path)
    out_path.write_text(xml)
    return out_path


# The above represents the core research infrastructure