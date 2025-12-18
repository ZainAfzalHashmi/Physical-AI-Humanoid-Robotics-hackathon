import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: '🧠 Conceptual & Elegant',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        "Engineering the Digital Synapses of Modern Humanoid Robotics"
      </>
    ),
  },
  {
    title: '🚀 Action-Oriented',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        "Mastering ROS 2: Building the Foundation for Physical AI and Autonomous Motion"
      </>
    ),
  },
  {
    title: '📖 Educational',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
       "From Nodes to Narratives: Building Intelligent Controllers with rclpy and URDF"
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
