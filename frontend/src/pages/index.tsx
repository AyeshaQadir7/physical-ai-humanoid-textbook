import type {ReactNode} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HeroSection from '@site/src/components/HeroSection';
import WhyPhysicalAI from '@site/src/components/WhyPhysicalAI';
import WhatYouLearn from '@site/src/components/WhatYouLearn';
import ModuleCards from '@site/src/components/ModuleCards';
import FinalCTA from '@site/src/components/FinalCTA';

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description={siteConfig.tagline}>
      <main>
        <HeroSection />
        <WhyPhysicalAI />
        <WhatYouLearn />
        <ModuleCards />
        <FinalCTA />
      </main>
    </Layout>
  );
}
